/*
 * Minimal MAX32655 + MAX30009 bioimpedance acquisition.
 *
 * Deliberately has no BLE stack, no SD card and no CLI: just the AFE, an
 * interrupt, and samples printed to the console. It exists as a reproducer small
 * enough to trust. If this runs indefinitely without freezing, the fault in the
 * full firmware lives in one of the subsystems that is absent here, and they can
 * be restored one at a time.
 *
 * The two structural rules it keeps, which the full firmware broke:
 *
 *   1. The ISR sets a flag and returns. It never touches the SPI bus. The MSDK's
 *      blocking transaction spins on MST_DONE with no timeout
 *      (spi_reva1.c: MXC_SPI_RevA1_MasterTransaction), so a transaction started
 *      from an ISR while another is in flight can wedge the first one forever --
 *      an unrecoverable freeze with no watchdog behind it.
 *   2. Only the main loop talks to the AFE. There is exactly one place where SPI
 *      transactions originate.
 *
 * Output is CSV on the console at 115200 baud:
 *   timestamp_ms, Q_ohm, I_ohm, tone_Hz
 *
 * Lines beginning with '#' are commentary, not data.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/***** Definitions *****/

/** MAX30009 INTB, active low, on P0.25. */
#define AFE_INT_PORT MXC_GPIO0
#define AFE_INT_PIN MXC_GPIO_PIN_25

/** Seconds between health lines on the console. 0 disables them. */
#define STATUS_PERIOD_S 10

/***** State *****/

/* Set by the ISR, cleared by the loop. */
static volatile bool afe_int_pending;

/* Counts edges, so a missed service is visible rather than silent. */
static volatile uint32_t afe_int_count;

/***** Interrupt *****/

/**
 * @brief INTB handler. Sets a flag; touches nothing else.
 *
 * The original did a blocking regRead(0x00) here, at the ADC sample rate, in
 * interrupt context. Besides costing ~99 us per interrupt, it could collide with
 * a transaction already running in the main loop and hang both.
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
      .pad = MXC_GPIO_PAD_PULL_UP, /* INTB is open-drain, active low */
      .vssel = MXC_GPIO_VSSEL_VDDIO};

  MXC_GPIO_Config(&cfg);
  MXC_GPIO_RegisterCallback(&cfg, afeISR, NULL);
  MXC_GPIO_IntConfig(&cfg, MXC_GPIO_INT_FALLING);
  MXC_GPIO_ClearFlags(AFE_INT_PORT, AFE_INT_PIN);
  MXC_GPIO_EnableInt(AFE_INT_PORT, AFE_INT_PIN);

  NVIC_ClearPendingIRQ(MXC_GPIO_GET_IRQ(0));
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0));
}

/***** Main *****/

int main(void)
{
  int err;

  printf("\n# --- minimal BioZ acquisition ---\n");
  printf("# no BLE, no SD card, no CLI\n");

  if ((err = initSPI()) != E_NO_ERROR)
  {
    printf("# FATAL SPI init failed: %d\n", err);
    return err;
  }

  /* Identity check before trusting anything else the bus says. */
  int part_id = regRead(PART_ID_REGISTER);
  printf("# PART_ID (0x%02X) = 0x%02X\n", PART_ID_REGISTER, part_id);
  if (part_id < 0)
  {
    printf("# FATAL no SPI response -- check wiring, power and chip select\n");
    return E_COMM_ERR;
  }

  BIAsettings();
  setFreq(0); /* start on the low tone */

  /* Cache the ohm coefficient and tone frequency once, so the decode path does
   * no register reads. */
  biozRefreshCache();

  printf("# SR_BIOZ        = %.2f sps\n", sr_bioz);
  printf("# tone           = %.0f Hz\n", getBiozFreq());
  printf("# gain           = %.0f V/V\n", getBiozGain());
  printf("# drive          = %.2f uA peak\n", getBiozCurrent_uA());
  printf("# ohm coeff      = %.9f ohm/count\n", getBiozOhmCoeffCached());
  printf("# discard/keep   = %d/%d per tone visit\n", BIOZ_SETTLE_DISCARD,
         BIOZ_KEEP_PER_VISIT);
  printf("# effective rate = %.1f sps total\n",
         sr_bioz * BIOZ_KEEP_PER_VISIT / (BIOZ_SETTLE_DISCARD + BIOZ_KEEP_PER_VISIT));

  /* Readback of the registers most worth confirming. */
  printf("# 0x0D FIFO_A_FULL=0x%02X  0x80 INT_EN=0x%02X  0x20 CFG1=0x%02X\n",
         regRead(0x0D), regRead(0x80), regRead(0x20));
  printf("# 0x21=0x%02X 0x22=0x%02X 0x24=0x%02X 0x25=0x%02X 0x41=0x%02X\n",
         regRead(0x21), regRead(0x22), regRead(0x24), regRead(0x25), regRead(0x41));

  setupAfeInterrupt();

  /* Start the FIFO empty, so the first interrupt means a full watermark of fresh
   * samples rather than whatever accumulated during configuration. */
  changeReg(0x0E, 1, 4, 1); /* FLUSH_FIFO */
  regRead(STATUS_1_REGISTER);

  biozEnable();

  printf("# timestamp_ms,Q_ohm,I_ohm,tone_Hz\n");

  uint32_t last_status_kept = 0;
  uint32_t loops_since_int = 0;

  while (1)
  {
    if (afe_int_pending)
    {
      afe_int_pending = false;
      loops_since_int = 0;

      /*
       * Clears A_FULL so INTB deasserts and the next almost-full condition can
       * produce a fresh falling edge. Must happen, and must happen here rather
       * than in the ISR.
       */
      regRead(STATUS_1_REGISTER);

      fifoBurst(true);

      /*
       * If the FIFO is still above the watermark, go round again without waiting
       * for another edge. INTB is level-asserting while the condition holds but
       * the GPIO is edge-triggered, so a missed edge would otherwise leave the
       * FIFO filling unattended until it overflowed.
       */
      if (fifoReadLevel(NULL) >= 16)
      {
        afe_int_pending = true;
      }
    }
    else
    {
      loops_since_int++;
    }

#if STATUS_PERIOD_S > 0
    /* Periodic health line, keyed to kept samples so it costs nothing when the
     * stream has stopped -- and its absence is itself the symptom. */
    uint32_t kept = biozKeptCount();
    uint32_t period = (uint32_t)(sr_bioz * STATUS_PERIOD_S * BIOZ_KEEP_PER_VISIT /
                                 (BIOZ_SETTLE_DISCARD + BIOZ_KEEP_PER_VISIT));
    if (period > 0 && kept >= last_status_kept + period)
    {
      last_status_kept = kept;
      printf("# ints=%lu decoded=%lu kept=%lu lost=%lu tagerr=%lu fifo=%u\n",
             (unsigned long)afe_int_count, (unsigned long)biozDecodedCount(),
             (unsigned long)kept, (unsigned long)fifoLostSamples(),
             (unsigned long)fifoTagErrors(), (unsigned)fifoReadLevel(NULL));
    }

    /*
     * Watchdog-by-observation: if the loop spins for a long time with no
     * interrupt, say so. A silent console is ambiguous -- this distinguishes "the
     * AFE stopped asserting INTB" from "the firmware is wedged", which is exactly
     * the distinction the full firmware could not make.
     */
    if (loops_since_int == 20000000u)
    {
      printf("# WARNING no INTB for a long while: ints=%lu fifo=%u status=0x%02X\n",
             (unsigned long)afe_int_count, (unsigned)fifoReadLevel(NULL),
             regRead(STATUS_1_REGISTER));
    }
#endif
  }

  /* Not reached. */
}
