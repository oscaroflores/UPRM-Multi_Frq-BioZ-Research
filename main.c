/*
This is the main file for communication done between the MAX32655 and the
MAX30009 to measure BioImpedance.


The current setup set out good frequencies for BioImpedance, but there are functions to
easily change the settings to measure at different rates or to measure different
vital signs depending on necessities

*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "i2c.h"
#include "gpio.h"
#include "tmr.h"
#include "dats_api.h"
#include "MAX30009.h"
#include "MAX32655.h"
#include "bioZ.h"
#include "board.h"
#include "cli.h"
#include "dma.h"
#include "gpio.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "sdhc.h"
#include "spi.h"
#include "spiFunctions.h"
#include "rtc.h"
#include "tmr.h"
#include "uart.h"
#include "user-cli.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "wsf_msg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_heap.h"
#include "wsf_cs.h"
#include "wsf_timer.h"
#include "wsf_os.h"

#include "sec_api.h"
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "app_terminal.h"
#include "wut.h"
#include "rtc.h"
#include "trimsir_regs.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_init_api.h"
#endif

#include "pal_bb.h"
#include "pal_cfg.h"
#include "app_ui.h"
/***** Definitions *****/
#define SPI_SPEED 4000000 // Bit Rate
/*** WIRING / ELECTRICAL TOGGLES ***/
#define INT_ACTIVE_LOW 0  // 0: active-high (LSM6DSL default), 1: active-low
#define INT_OPEN_DRAIN 0  // 0: push-pull (default), 1: open-drain (add pull-up)
#define USE_VDDIOH_I2C2 0 // 1 if your I2C bus is 3.3V and you need VDDIOH on P0.30/31

/*** I2C / LSM6DSL ***/
#define I2C_MASTER MXC_I2C2
#define I2C_FREQ 1000000
#define LSM6DSL_ADDR 0x6A // use 0x6A if SA0/SDO is low

// Registers
#define WHO_AM_I_REG 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define INT1_CTRL 0x0D
#define DRDY_PULSE_CFG_G 0x0B
#define STATUS_REG 0x1E
#define OUTX_L_XL 0x28
/*** INT1 pin: P1_8 (GPIO1.8) ***/
#define IMU_INT_PORT MXC_GPIO1
#define IMU_INT_PIN MXC_GPIO_PIN_8 // mask bit for pin 8
#define IMU_INT_MASK IMU_INT_PIN
// Board Selection
#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U
#define DEFAULT_TX_POWER 0 /* dBm */
#define MXC_BASE_WUT0 ((uint32_t)0x40006400UL)
#define MXC_WUT0 ((mxc_wut_regs_t *)MXC_BASE_WUT0)
#define IMU_BUF_SIZE 128
#define IMU_FLUSH_BATCH 1
#define IMU_LOG_DECIM 3
#define IMU_BLE_DECIM 10
#define IMU_MAX_DRDY_PER_LOOP 1
#define IMU_ODR_HZ 104

/***** Globals *****/
extern volatile bool recordingIMU;
static uint8_t imu_err_streak = 0;
extern FIL imuFile;

static volatile uint16_t imu_drdy_count = 0;
bool current_freq = 0;
volatile bool recording = false;
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
volatile uint16_t bioz_irq_count = 0;
extern uint32_t sample_interval_us;
extern sample_index;
int samples_discarded;
static wsfBufPoolDesc_t mainPoolDesc[] = {{16, 8}, {32, 4}, {192, 8}, {256, 16}};
bool sample_ready = 0;
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
static LlRtCfg_t mainLlRtCfg;
#endif

volatile int wutTrimComplete;

typedef struct
{
  uint32_t t_ms;
  int16_t ax;
  int16_t ay;
  int16_t az;
} imu_sample_t;

static imu_sample_t imu_buf[IMU_BUF_SIZE];
static uint16_t imu_buf_head = 0;
static uint16_t imu_buf_tail = 0;
static uint32_t imu_sample_total = 0;
static uint32_t imu_sample_index = 0;

extern void StackInitDats(void);

/*************************************************************************************************/
/*!
 *  \brief  Initialize WSF.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mainWsfInit(void)
{
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
  /* +12 for message headroom, + 2 event header, +255 maximum parameter length. */
  const uint16_t maxRptBufSize = 12 + 2 + 255;

  /* +12 for message headroom, +4 for header. */
  const uint16_t aclBufSize = 12 + mainLlRtCfg.maxAclLen + 4 + BB_DATA_PDU_TAILROOM;

  /* Adjust buffer allocation based on platform configuration. */
  mainPoolDesc[2].len = maxRptBufSize;
  mainPoolDesc[2].num = mainLlRtCfg.maxAdvReports;
  mainPoolDesc[3].len = aclBufSize;
  mainPoolDesc[3].num = mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs;
#endif

  const uint8_t numPools = sizeof(mainPoolDesc) / sizeof(mainPoolDesc[0]);

  uint16_t memUsed;
  WsfCsEnter();
  memUsed = WsfBufInit(numPools, mainPoolDesc);
  WsfHeapAlloc(memUsed);
  WsfCsExit();

  WsfOsInit();
  WsfTimerInit();
#if (WSF_TOKEN_ENABLED == TRUE) || (WSF_TRACE_ENABLED == TRUE)
  WsfTraceRegisterHandler(WsfBufIoWrite);
  WsfTraceEnable(TRUE);
#endif
}

/*************************************************************************************************/
/*!
 *  \fn     wutTrimCb
 *
 *  \brief  Callback function for the WUT 32 kHz crystal trim.
 *
 *  \param  err    Error code from the WUT driver.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wutTrimCb(int err)
{
  if (err != E_NO_ERROR)
  {
    APP_TRACE_INFO1("32 kHz trim error %d\n", err);
  }
  else
  {
    APP_TRACE_INFO1("32kHz trimmed to 0x%x", (MXC_TRIMSIR->rtc & MXC_F_TRIMSIR_RTC_X1TRIM) >>
                                                 MXC_F_TRIMSIR_RTC_X1TRIM_POS);
  }
  wutTrimComplete = 1;
}

/*************************************************************************************************/
/*!
 *  \fn     setAdvTxPower
 *
 *  \brief  Set the default advertising TX power.
 *
 *  \return None.
 */
/*************************************************************************************************/
static int i2c_write(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  mxc_i2c_req_t req = {
      .i2c = I2C_MASTER, .addr = LSM6DSL_ADDR, .tx_buf = buf, .tx_len = 2, .rx_buf = NULL, .rx_len = 0, .restart = 0, .callback = NULL};
  int r = MXC_I2C_MasterTransaction(&req);
  printf("[DEBUG] Write reg 0x%02X = 0x%02X -> %s\n", reg, val, r == 0 ? "OK" : "FAIL");
  return r;
}
static int i2c_read(uint8_t reg, uint8_t *data, int len)
{
  mxc_i2c_req_t req = {
      .i2c = I2C_MASTER, .addr = LSM6DSL_ADDR, .tx_buf = &reg, .tx_len = 1, .rx_buf = data, .rx_len = len, .restart = 1, .callback = NULL};
  int rc = MXC_I2C_MasterTransaction(&req);
  if (rc == E_COMM_ERR || rc == -7)
  {
    // printf("[I2C RECOVER] bus stuck, clearing...\n");

    // --- Bus clear ---
    mxc_gpio_cfg_t scl = {
        .port = MXC_GPIO0,
        .mask = MXC_GPIO_PIN_30,
        .func = MXC_GPIO_FUNC_OUT,
        .vssel = MXC_GPIO_VSSEL_VDDIO};
    MXC_GPIO_Config(&scl);
    for (int i = 0; i < 9; i++)
    {
      MXC_GPIO_OutClr(MXC_GPIO0, MXC_GPIO_PIN_30);
      MXC_Delay(MXC_DELAY_USEC(5));
      MXC_GPIO_OutSet(MXC_GPIO0, MXC_GPIO_PIN_30);
      MXC_Delay(MXC_DELAY_USEC(5));
    }

    // --- Reinit controller ---
    MXC_I2C_Shutdown(I2C_MASTER);
    MXC_I2C_Init(I2C_MASTER, 1, 0);
    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
    MXC_GPIO_ClearFlags(IMU_INT_PORT, IMU_INT_MASK);
    MXC_GPIO_EnableInt(IMU_INT_PORT, IMU_INT_MASK);

    return -1;
  }
  return rc;
}

/*** GPIO ISR ***/
static void imuISR(void *unused)
{
  MXC_GPIO_ClearFlags(IMU_INT_PORT, IMU_INT_MASK);
  if (imu_drdy_count < UINT16_MAX)
  {
    imu_drdy_count++;
  }
}
void GPIO1_IRQHandler(void) { MXC_GPIO_Handler(1); }

static void setup_gpio_int_config_only(void)
{
  mxc_gpio_cfg_t pin = {
      .port = IMU_INT_PORT,
      .mask = IMU_INT_MASK,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_NONE,
      .vssel = MXC_GPIO_VSSEL_VDDIO};
  MXC_GPIO_Config(&pin);
  MXC_GPIO_ClearFlags(pin.port, pin.mask);
  MXC_GPIO_RegisterCallback(&pin, imuISR, NULL);

  // Latched DRDY => use LEVEL-HIGH (active-low? use MXC_GPIO_INT_LOW and set H_LACTIVE)
  MXC_GPIO_IntConfig(&pin, MXC_GPIO_INT_RISING);
  // DO NOT: MXC_GPIO_EnableInt(...) yet
  // DO NOT: NVIC_EnableIRQ(...) yet
}

// --- After sensor config is done, THEN enable GPIO + NVIC ---
static void enable_imu_gpio_irq(void)
{
  MXC_GPIO_ClearFlags(IMU_INT_PORT, IMU_INT_MASK);
  MXC_GPIO_EnableInt(IMU_INT_PORT, IMU_INT_MASK);
  NVIC_ClearPendingIRQ(MXC_GPIO_GET_IRQ(1));
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(1));
}

static inline uint16_t imu_buffer_depth(void)
{
  if (imu_buf_head >= imu_buf_tail)
    return imu_buf_head - imu_buf_tail;
  return (IMU_BUF_SIZE - imu_buf_tail) + imu_buf_head;
}

static inline void imu_buffer_add(uint32_t t_ms, int16_t ax, int16_t ay, int16_t az)
{
  uint16_t next = (imu_buf_head + 1) % IMU_BUF_SIZE;
  if (next == imu_buf_tail)
  {
    imu_buf_tail = (imu_buf_tail + 1) % IMU_BUF_SIZE; // overwrite oldest
  }

  imu_buf[imu_buf_head].t_ms = t_ms;
  imu_buf[imu_buf_head].ax = ax;
  imu_buf[imu_buf_head].ay = ay;
  imu_buf[imu_buf_head].az = az;
  imu_buf_head = next;
}

static inline bool imu_buffer_pop(imu_sample_t *out)
{
  if (imu_buf_head == imu_buf_tail)
    return false;
  *out = imu_buf[imu_buf_tail];
  imu_buf_tail = (imu_buf_tail + 1) % IMU_BUF_SIZE;
  return true;
}
void reset_imu_logging_state(void)
{
  imu_buf_head = 0;
  imu_buf_tail = 0;
  imu_drdy_count = 0;
  imu_sample_total = 0;
  imu_sample_index = 0;
}
void setAdvTxPower(void)
{
  LlSetAdvTxPower(DEFAULT_TX_POWER);
}
void buttonISR(void *unused)
{
  // 1. Clear the GPIO interrupt flag
  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_2);

  if (!recording)
  {
    datsSendData(AppConnIsOpen(), "startPhys\n", sizeof("startPhys\n") - 1);
    recording = 1;
  }
  else
  {
    datsSendData(AppConnIsOpen(), "stopPhys\n", sizeof("stopPhys\n") - 1);
    recording = 0;
  }
}

void setupButtonInterrupt()
{
  mxc_gpio_cfg_t intaPin = {
      .port = MXC_GPIO0,
      .mask = MXC_GPIO_PIN_2,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP,  // <-- Enable internal pull-up
      .vssel = MXC_GPIO_VSSEL_VDDIO // or VDDIOH if you use 3.3V IO
  };

  MXC_GPIO_Config(&intaPin);

  MXC_GPIO_RegisterCallback(&intaPin, buttonISR, NULL);
  MXC_GPIO_IntConfig(&intaPin, MXC_GPIO_INT_FALLING); // Detect falling edge
  MXC_GPIO_EnableInt(intaPin.port, intaPin.mask);

  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0)); // Enable NVIC interrupt for GPIO0 block
}
void sensorISR(void *unused)
{

  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_25); // Clear interrupt

  regRead(0x00);
  if (bioz_irq_count < UINT16_MAX)
    bioz_irq_count++;
}

void setupMax30009Interrupt(void)
{
  mxc_gpio_cfg_t intbPin = {
      .port = MXC_GPIO0,
      .mask = MXC_GPIO_PIN_25,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP,  // <-- Enable internal pull-up
      .vssel = MXC_GPIO_VSSEL_VDDIO // or VDDIOH if you use 3.3V IO
  };

  MXC_GPIO_Config(&intbPin);

  MXC_GPIO_RegisterCallback(&intbPin, sensorISR, NULL);
  MXC_GPIO_IntConfig(&intbPin, MXC_GPIO_INT_FALLING); // Detect falling edge
  MXC_GPIO_EnableInt(intbPin.port, intbPin.mask);

  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0)); // Enable NVIC interrupt for GPIO0 block
}

int main(void)
{
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
  /* Configurations must be persistent. */
  static BbRtCfg_t mainBbRtCfg;

  PalBbLoadCfg((PalBbCfg_t *)&mainBbRtCfg);
  LlGetDefaultRunTimeCfg(&mainLlRtCfg);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
  /* Set 5.0 requirements. */
  mainLlRtCfg.btVer = LL_VER_BT_CORE_SPEC_5_0;
#endif
  PalCfgLoadData(PAL_CFG_ID_LL_PARAM, &mainLlRtCfg.maxAdvSets, sizeof(LlRtCfg_t) - 9);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
  PalCfgLoadData(PAL_CFG_ID_BLE_PHY, &mainLlRtCfg.phy2mSup, 4);
#endif

  /* Set the 32k sleep clock accuracy into one of the following bins, default is 20
    HCI_CLOCK_500PPM
    HCI_CLOCK_250PPM
    HCI_CLOCK_150PPM
    HCI_CLOCK_100PPM
    HCI_CLOCK_75PPM
    HCI_CLOCK_50PPM
    HCI_CLOCK_30PPM
    HCI_CLOCK_20PPM
  */
  mainBbRtCfg.clkPpm = 20;

  /* Set the default connection power level */
  mainLlRtCfg.defTxPwrLvl = DEFAULT_TX_POWER;
#endif

  uint32_t memUsed;
  WsfCsEnter();
  // memUsed = WsfBufIoUartInit(WsfHeapGetFreeStartAddress(), PLATFORM_UART_TERMINAL_BUFFER_SIZE);
  WsfHeapAlloc(memUsed);
  WsfCsExit();

  mainWsfInit();
  // AppTerminalInit();

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)

  WsfCsEnter();
  LlInitRtCfg_t llCfg = {.pBbRtCfg = &mainBbRtCfg,
                         .wlSizeCfg = 4,
                         .rlSizeCfg = 4,
                         .plSizeCfg = 4,
                         .pLlRtCfg = &mainLlRtCfg,
                         .pFreeMem = WsfHeapGetFreeStartAddress(),
                         .freeMemAvail = WsfHeapCountAvailable()};

  memUsed = LlInit(&llCfg);
  WsfHeapAlloc(memUsed);
  WsfCsExit();

  bdAddr_t bdAddr;
  PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
  LlSetBdAddr((uint8_t *)&bdAddr);

  /* Start the 32 MHz crystal and the BLE DBB counter to trim the 32 kHz crystal */
  PalBbEnable();

  /* Output buffered square wave of 32 kHz clock to GPIO */
  // MXC_RTC_SquareWaveStart(MXC_RTC_F_32KHZ);

  /* Execute the trim procedure */
  wutTrimComplete = 0;
  MXC_WUT_TrimCrystalAsync(MXC_WUT0, wutTrimCb);
  while (!wutTrimComplete)
  {
  }

  /* Shutdown the 32 MHz crystal and the BLE DBB */
  PalBbDisable();
#endif

  StackInitDats();
  DatsStart();
  int err;

  printf("START\n");
  MXC_I2C_Init(I2C_MASTER, 1, 0);
  MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

  // Probe
  uint8_t who = 0;
  i2c_read(WHO_AM_I_REG, &who, 1);
  printf("WHO_AM_I: 0x%02X\n", who);

  setup_gpio_int_config_only();

  // Reset + config
  i2c_write(CTRL3_C, 0x01);
  MXC_Delay(MXC_DELAY_MSEC(20)); // <- give it time after reset
  i2c_write(CTRL3_C, 0x44);      // BDU=1, IF_INC=1
  i2c_write(CTRL2_G, 0x00);      // gyro off
  i2c_write(CTRL1_XL, 0x40);     // accel 104Hz
  i2c_write(DRDY_PULSE_CFG_G, 0x80); // pulsed DRDY
  i2c_write(INT1_CTRL, 0x01); // XL_DRDY -> INT1

  uint8_t st = 0;
  i2c_read(STATUS_REG, &st, 1);
  enable_imu_gpio_irq();
  MXC_Delay(MXC_DELAY_MSEC(100));

  if ((err = MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), user_commands,
                          num_user_commands)) != E_NO_ERROR)
  {
    return err;
  }
  initSPI();             // Setup SPI
  BIAsettings();         // Set up correct sensor registers for SFBIA
  setFreq(current_freq); // Set frequency to 150 kHz
  setupButtonInterrupt();
  setupMax30009Interrupt();

  // --- Timer 0: Continuous timestamp tracking ---
  mxc_tmr_cfg_t tmr0_cfg;
  tmr0_cfg.pres = TMR_PRES_1;
  tmr0_cfg.mode = TMR_MODE_CONTINUOUS;
  tmr0_cfg.bitMode = TMR_BIT_MODE_32;
  tmr0_cfg.clock = MXC_TMR_APB_CLK;
  tmr0_cfg.cmp_cnt = 0xFFFFFFFF;
  tmr0_cfg.pol = 0;

  MXC_TMR_Init(MXC_TMR0, &tmr0_cfg, false);
  MXC_TMR_Start(MXC_TMR0);
  sample_interval_us = getSampleInterval();

  mxc_tmr_cfg_t timer1_cfg;
  timer1_cfg.pres = TMR_PRES_1;
  timer1_cfg.mode = TMR_MODE_CONTINUOUS;
  timer1_cfg.bitMode = TMR_BIT_MODE_32;
  timer1_cfg.clock = MXC_TMR_ISO_CLK;
  timer1_cfg.cmp_cnt = 0xFFFFFFFF;
  timer1_cfg.pol = 0;

  MXC_TMR_Init(MXC_TMR1, &timer1_cfg, false);
  MXC_TMR_Start(MXC_TMR1);
  static uint32_t last_call = 0;
  setupMax30009Interrupt();
  const double LSB_G = 0.061 / 1000.0;

  while (1)
  {
    WsfTimerSleepUpdate();
    // uint32_t start = MXC_TMR_GetCount(MXC_TMR1);

    wsfOsDispatcher();

    /* ---------- BIOZ SERVICE ---------- */
    while (bioz_irq_count)
    {
      bioz_irq_count--;
      if (sample_ready)
      {
        sample_ready = 0;
        spiBurst(current_freq);
        current_freq = !current_freq;
        setFreq(current_freq);
      }
      else
      {
        sample_index++;
        samples_discarded++;
        spiBurstnoPrint();
        if (samples_discarded == 7)
        {
          sample_ready = 1;
          samples_discarded = 0;
        }
      }
    }
    /* ---------- IMU SERVICE ---------- */
    if (bioz_irq_count == 0)
    {
      uint16_t imu_to_service = imu_drdy_count;
      if (imu_to_service > IMU_MAX_DRDY_PER_LOOP)
        imu_to_service = IMU_MAX_DRDY_PER_LOOP;
      while (imu_to_service--)
      {
        imu_drdy_count--;
        if (!recordingIMU)
        {
          continue;
        }
      uint8_t raw_accel[6];
      int rc = i2c_read(OUTX_L_XL, raw_accel, 6);
      if (rc != 0)
      {
        // printf("[I2C ERR] rc=%d\n", rc);
        imu_err_streak++;
        if (imu_err_streak >= 3)
          {
            // printf("Reinitializing I2C...\n");
            MXC_I2C_Shutdown(I2C_MASTER);
            MXC_I2C_Init(I2C_MASTER, 1, 0);
            MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
            imu_err_streak = 0;
          }
      }
      else
      {
        imu_err_streak = 0;

        int16_t ax = (int16_t)((raw_accel[1] << 8) | raw_accel[0]);
        int16_t ay = (int16_t)((raw_accel[3] << 8) | raw_accel[2]);
        int16_t az = (int16_t)((raw_accel[5] << 8) | raw_accel[4]);
        uint32_t t_ms = (imu_sample_index * 1000) / IMU_ODR_HZ;
        imu_sample_index++;
        imu_buffer_add(t_ms, ax, ay, az);
      }
    }
    }

    /* Flush IMU buffer in batches to keep ISR light */
    if (recordingIMU && bioz_irq_count == 0)
    {
      uint16_t pending = imu_buffer_depth();
      if (pending)
      {
        uint16_t to_flush = pending > IMU_FLUSH_BATCH ? IMU_FLUSH_BATCH : pending;
        char log_chunk[256];
        uint16_t chunk_len = 0;

        while (to_flush--)
        {
          imu_sample_t s;
          if (!imu_buffer_pop(&s))
            break;

          imu_sample_total++;

          // Format accel-only line
          char line_ble[64];
          int line_ble_len = snprintf(line_ble, sizeof(line_ble),
                                      "%lu,%.2f,%.2f,%.2f\n",
                                      (unsigned long)s.t_ms,
                                      (double)s.ax * LSB_G, (double)s.ay * LSB_G, (double)s.az * LSB_G);
          if (line_ble_len <= 0 || line_ble_len >= (int)sizeof(line_ble))
          {
            continue;
          }

          if (chunk_len + line_ble_len >= sizeof(log_chunk))
          {
            UINT written;
            int err = f_write(&imuFile, log_chunk, chunk_len, &written);
            if (err != FR_OK || written != chunk_len)
            {
              printf("Write failed: %s\n", FF_ERRORS[err]);
              return err;
            }
            chunk_len = 0;
          }
          // BLE decimation
          if ((IMU_BLE_DECIM <= 1) || ((imu_sample_total % IMU_BLE_DECIM) == 0))
          {
            datsSendData(AppConnIsOpen(), line_ble, line_ble_len);
          }

          // Log accel-only to SD
          memcpy(log_chunk + chunk_len, line_ble, line_ble_len);
          chunk_len += line_ble_len;
        }

        if (chunk_len)
        {
          UINT written;
          int err = 0;
          if ((err = f_write(&imuFile, log_chunk, chunk_len, &written)) != FR_OK || written != chunk_len)
          {
            printf("Write failed: %s\n", FF_ERRORS[err]);
            return err;
          }
        }
      }
    }

    if (!WsfOsActive())
    {
      WsfTimerSleep();
    }
  }

  printf("error count = %d\n", errCnt);

  shutdownSPI();
  umount();

  printf("Finished\n");

  return E_NO_ERROR;
}
