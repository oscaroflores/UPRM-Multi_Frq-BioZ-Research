/*
 * Minimal MAX32655 + MAX30009 bioimpedance acquisition.
 *
 * No BLE, no SD card, no CLI: just the AFE, and samples printed to the console.
 * A reproducer small enough to trust, so that if it runs the fault in the full
 * firmware lies in one of the subsystems absent here.
 *
 * Structural rules, both broken by the full firmware:
 *
 *   1. The ISR sets a flag and returns; it never touches SPI. The MSDK's blocking
 *      transaction ends in `while (!(intfl & MST_DONE)) {}` with no timeout
 *      (spi_reva1.c), so a transaction started from an ISR while another is in
 *      flight can wedge the first one permanently.
 *   2. Only the main loop talks to the AFE -- one place where transactions start.
 *
 * It is also built so it cannot fail silently. Every configuration write is read
 * back and checked, a heartbeat prints unconditionally so a wedge is
 * distinguishable from an idle wait, and if INTB never arrives the loop falls
 * back to polling the FIFO -- which separates "the AFE is not sampling" from
 * "the interrupt is not getting through".
 *
 * Console, 115200 baud. Lines starting with '#' are commentary; the rest is CSV:
 *   timestamp_ms, Q_ohm, I_ohm, tone_Hz
 */

#include "bioZ.h"
#include "spiFunctions.h"

#include "MAX30009.h"
#include "board.h"
#include "gpio.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "tmr.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/***** Definitions *****/

/** MAX30009 INTB, open-drain active low (INT_OCFG = 0), on P0.25. */
#define AFE_INT_PORT MXC_GPIO0
#define AFE_INT_PIN MXC_GPIO_PIN_25

/** FIFO samples that should be waiting when A_FULL asserts (256 - 0xF0). */
#define AFE_WATERMARK_SAMPLES 16

/** Heartbeat interval. Prints unconditionally, so silence means wedged. */
#define HEARTBEAT_MS 2000

/** If INTB has been quiet this long, poll the FIFO instead of waiting. */
#define POLL_FALLBACK_MS 250

/** Timer for the timebase. TMR_PRES_1 on the APB clock, 32-bit, continuous. */
#define TIMEBASE_TMR MXC_TMR0

/***** State *****/

static volatile bool afe_int_pending;
static volatile uint32_t afe_int_count;

static uint32_t tmr_ticks_per_ms;
static uint32_t poll_services;
static bool interrupt_seen;

/***** Timebase *****/

static void timebaseInit(void)
{
  mxc_tmr_cfg_t cfg;

  cfg.pres = TMR_PRES_1;
  cfg.mode = TMR_MODE_CONTINUOUS;
  cfg.bitMode = TMR_BIT_MODE_32;
  cfg.clock = MXC_TMR_APB_CLK;
  cfg.cmp_cnt = 0xFFFFFFFF;
  cfg.pol = 0;

  MXC_TMR_Init(TIMEBASE_TMR, &cfg, false);
  MXC_TMR_Start(TIMEBASE_TMR);

  tmr_ticks_per_ms = PeripheralClock / 1000u;
}

/** @brief Milliseconds since boot. Wraps after ~49 days at 1 tick/us. */
static uint32_t nowMs(void)
{
  if (tmr_ticks_per_ms == 0)
  {
    return 0;
  }
  return MXC_TMR_GetCount(TIMEBASE_TMR) / tmr_ticks_per_ms;
}

/***** Interrupt *****/

/**
 * @brief INTB handler. Sets a flag and nothing else.
 *
 * The original did a blocking regRead(0x00) here, in interrupt context, at the
 * ADC sample rate.
 */
static void afeISR(void *unused)
{
  (void)unused;

  MXC_GPIO_ClearFlags(AFE_INT_PORT, AFE_INT_PIN);
  afe_int_pending = true;
  afe_int_count++;
}

void GPIO0_IRQHandler(void)
{
  MXC_GPIO_Handler(0);
}

static void setupAfeInterrupt(void)
{
  mxc_gpio_cfg_t cfg = {
      .port = AFE_INT_PORT,
      .mask = AFE_INT_PIN,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP, /* INTB is open-drain, so a pull-up is required */
      .vssel = MXC_GPIO_VSSEL_VDDIO};

  MXC_GPIO_Config(&cfg);
  MXC_GPIO_RegisterCallback(&cfg, afeISR, NULL);
  MXC_GPIO_IntConfig(&cfg, MXC_GPIO_INT_FALLING);
  MXC_GPIO_ClearFlags(AFE_INT_PORT, AFE_INT_PIN);
  MXC_GPIO_EnableInt(AFE_INT_PORT, AFE_INT_PIN);

  NVIC_ClearPendingIRQ(MXC_GPIO_GET_IRQ(0));
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0));
}

/***** Configuration check *****/

/**
 * @brief Read a register back and report whether it holds what was written.
 *
 * Silent misconfiguration is the failure mode that wastes the most time: the AFE
 * simply produces nothing and there is no way to tell why. changeReg() already
 * skips its write when the readback fails, so a broken bus shows up here too.
 */
static bool checkReg(uint8_t addr, uint8_t expect, const char *name)
{
  int got = regRead(addr);

  if (got < 0)
  {
    printf("# FAIL 0x%02X %s: no response\n", addr, name);
    return false;
  }
  if ((uint8_t)got != expect)
  {
    printf("# MISMATCH 0x%02X %s: wrote 0x%02X, read 0x%02X\n", addr, name, expect,
           (unsigned)got);
    return false;
  }
  printf("# ok 0x%02X %s = 0x%02X\n", addr, name, (unsigned)got);
  return true;
}

/** @brief Service one FIFO batch. Only ever called from the main loop. */
static void serviceAfe(void)
{
  /* Clears A_FULL so INTB deasserts (INT_FCFG = 1 means the INT pin clears on a
   * status or FIFO read) and the next almost-full condition can produce a fresh
   * falling edge. */
  regRead(STATUS_1_REGISTER);

  fifoBurst(true);
}

/***** Main *****/

int main(void)
{
  int err;

  timebaseInit();

  printf("\n# --- minimal BioZ acquisition ---\n");
  printf("# no BLE, no SD card, no CLI\n");
  printf("# SPI inter-transaction delay: %d us\n", SPI_INTER_XACT_DELAY_US);

  if ((err = initSPI()) != E_NO_ERROR)
  {
    printf("# FATAL SPI init failed: %d\n", err);
    return err;
  }
  printf("# SPI up\n");

  /* Identity check before trusting anything else the bus says. */
  int part_id = regRead(PART_ID_REGISTER);
  if (part_id < 0)
  {
    printf("# FATAL no SPI response. Check wiring, power, and chip select.\n");
    return E_COMM_ERR;
  }
  printf("# PART_ID (0xFF) = 0x%02X\n", (unsigned)part_id);
  if (part_id == 0x00 || part_id == 0xFF)
  {
    printf("# WARNING PART_ID looks like a floating or stuck bus, not a device.\n");
  }

  BIAsettings();
  setFreq(0); /* start on the low tone */
  biozRefreshCache();

  /* --- Verify what actually landed. --- */
  printf("# --- configuration readback ---\n");
  bool cfg_ok = true;
  cfg_ok &= checkReg(0x12, 0x04, "PIN_FUNC (INT_FCFG=1, INT enabled)");
  cfg_ok &= checkReg(0x13, 0x00, "PIN_OUT  (INT_OCFG=0, open-drain low)");
  cfg_ok &= checkReg(0x0D, 0xF0, "FIFO_A_FULL (240 free -> trips at 16)");
  cfg_ok &= checkReg(0x80, 0x80, "INT_EN (A_FULL_EN only)");
  cfg_ok &= checkReg(0x21, 0x20, "DHPF/DLPF");
  cfg_ok &= checkReg(0x22, 0x28, "VDRV/IDRV");
  cfg_ok &= checkReg(0x24, 0x33, "AHPF/GAIN");
  cfg_ok &= checkReg(0x25, 0xCA, "AMP_RGE/BW");

  if (!cfg_ok)
  {
    printf("# WARNING configuration did not read back as written.\n");
    printf("# If several registers mismatch, SPI writes are not landing --\n");
    printf("# try rebuilding with -DSPI_INTER_XACT_DELAY_US=150.\n");
  }

  printf("# --- derived ---\n");
  printf("# SR_BIOZ        = %.2f sps\n", sr_bioz);
  printf("# tone           = %.0f Hz\n", getBiozFreq());
  printf("# gain           = %.0f V/V\n", getBiozGain());
  printf("# drive          = %.2f uA peak\n", getBiozCurrent_uA());
  printf("# ohm coeff      = %.9f ohm/count\n", getBiozOhmCoeffCached());
  printf("# discard/keep   = %d/%d per tone visit\n", BIOZ_SETTLE_DISCARD,
         BIOZ_KEEP_PER_VISIT);
  printf("# effective rate = %.1f sps\n",
         sr_bioz * BIOZ_KEEP_PER_VISIT / (BIOZ_SETTLE_DISCARD + BIOZ_KEEP_PER_VISIT));

  setupAfeInterrupt();
  printf("# INTB (P0.25) armed, level now %s\n",
         MXC_GPIO_InGet(AFE_INT_PORT, AFE_INT_PIN) ? "HIGH (idle)" : "LOW (asserted)");

  /* Start from an empty FIFO, so the first interrupt means a full watermark of
   * fresh samples rather than whatever accumulated during configuration. */
  changeReg(0x0E, 1, 4, 1); /* FLUSH_FIFO, self-clearing */
  regRead(STATUS_1_REGISTER);

  biozEnable();
  printf("# BIOZ enabled, 0x20 = 0x%02X (expect 0xA7)\n", (unsigned)regRead(0x20));
  printf("# timestamp_ms,Q_ohm,I_ohm,tone_Hz\n");

  uint32_t last_heartbeat_ms = nowMs();
  uint32_t last_service_ms = nowMs();

  while (1)
  {
    uint32_t t = nowMs();

    if (afe_int_pending)
    {
      afe_int_pending = false;
      interrupt_seen = true;
      last_service_ms = t;

      serviceAfe();

      /*
       * If the FIFO is still at or above the watermark, go again without waiting
       * for another edge: INTB is level-asserting while the condition holds, but
       * the GPIO is edge-triggered, so a missed edge would otherwise leave the
       * FIFO filling unattended until it overflowed.
       */
      if (fifoReadLevel(NULL) >= AFE_WATERMARK_SAMPLES)
      {
        afe_int_pending = true;
      }
    }
    else if ((uint32_t)(t - last_service_ms) >= POLL_FALLBACK_MS)
    {
      /*
       * Fallback. If the AFE is sampling but INTB never reaches the MCU, this
       * still produces data -- and the heartbeat's poll count versus interrupt
       * count then says which of the two is broken. At the configured rate a
       * watermark's worth of samples accumulates every ~62 ms, so reaching here
       * means roughly four intervals passed with no interrupt.
       */
      last_service_ms = t;

      uint16_t level = fifoReadLevel(NULL);
      if (level >= AFE_WATERMARK_SAMPLES)
      {
        poll_services++;
        serviceAfe();
      }
    }

    if ((uint32_t)(t - last_heartbeat_ms) >= HEARTBEAT_MS)
    {
      last_heartbeat_ms = t;

      int status = regRead(STATUS_1_REGISTER);
      int cfg1 = regRead(0x20);
      uint8_t ovf = 0;
      uint16_t level = fifoReadLevel(&ovf);

      /*
       * Prints unconditionally. If these lines keep coming, the firmware is
       * running and the question is why the AFE is quiet; if they stop, it is
       * wedged. The full firmware could not tell those apart.
       */
      printf("# t=%lus ints=%lu polls=%lu dec=%lu kept=%lu lost=%lu tagerr=%lu "
             "fifo=%u ovf=%u st=0x%02X cfg1=0x%02X intb=%d\n",
             (unsigned long)(t / 1000u), (unsigned long)afe_int_count,
             (unsigned long)poll_services, (unsigned long)biozDecodedCount(),
             (unsigned long)biozKeptCount(), (unsigned long)fifoLostSamples(),
             (unsigned long)fifoTagErrors(), (unsigned)level, (unsigned)ovf,
             (unsigned)status, (unsigned)cfg1,
             MXC_GPIO_InGet(AFE_INT_PORT, AFE_INT_PIN) ? 1 : 0);

      if (!interrupt_seen && biozDecodedCount() == 0 && level == 0)
      {
        printf("# DIAGNOSIS the FIFO is empty and INTB never fired: the AFE is "
               "not producing samples. Check 0x20 = 0xA7 above, and PLL lock.\n");
      }
      else if (!interrupt_seen && poll_services > 0)
      {
        printf("# DIAGNOSIS samples are arriving but INTB never fired: the AFE "
               "works, the interrupt path does not.\n");
      }
    }
  }

  /* Not reached. */
}
