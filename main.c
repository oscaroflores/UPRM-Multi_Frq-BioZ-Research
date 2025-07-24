/*
This is the main file for communication done between the MAX32655 and the
MAX30009 to measure BioImpedance.


The current setup set out good frequencies for BioImpedance, but there are functions to
easily change the settings to measure at different rates or to measure different
vital signs depending on necessities

*/
#include "tmr.h"

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
#include "i2c.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_init_api.h"
#endif

#include "pal_bb.h"
#include "pal_cfg.h"

#include "dats_api.h"
#include "app_ui.h"
/***** Definitions *****/
#define SPI_SPEED 1000000 // Bit Rate
#define I2C_MASTER MXC_I2C2
#define I2C_FREQ 100000
#define LSM6DSL_ADDR 0x6B
#define WHO_AM_I_REG 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL10_C 0x19
#define MASTER_CONFIG 0x1A
#define STATUS_REG 0x1E
#define OUTX_L_XL 0x28
#define OUTX_L_G 0x22
// Board Selection
#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U
#define DEFAULT_TX_POWER 0 /* dBm */
#define MXC_BASE_WUT0 ((uint32_t)0x40006400UL)
#define MXC_WUT0 ((mxc_wut_regs_t *)MXC_BASE_WUT0)
#define GYRO_CALIB_SAMPLES 50
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

/***** Globals *****/
bool current_freq = 0;
volatile bool recording = false;
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
bool interrupt = 0;
extern uint32_t sample_interval_us;
extern sample_index;
int samples_discarded;
static wsfBufPoolDesc_t mainPoolDesc[] = {{16, 8}, {32, 4}, {192, 8}, {256, 16}};
bool sample_ready = 0;
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
static LlRtCfg_t mainLlRtCfg;
#endif

volatile int wutTrimComplete;

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
int i2c_write(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {reg, value};
  mxc_i2c_req_t req = {
      .i2c = I2C_MASTER,
      .addr = LSM6DSL_ADDR,
      .tx_buf = buffer,
      .tx_len = 2,
      .rx_buf = NULL,
      .rx_len = 0,
      .restart = 0,
      .callback = NULL,
  };
  int result = MXC_I2C_MasterTransaction(&req);
  printf("[DEBUG] Write reg 0x%02X = 0x%02X -> %s\n", reg, value, result == 0 ? "OK" : "FAIL");
  return result;
}

int i2c_read(uint8_t reg, uint8_t *data, int len)
{
  mxc_i2c_req_t req = {
      .i2c = I2C_MASTER,
      .addr = LSM6DSL_ADDR,
      .tx_buf = &reg,
      .tx_len = 1,
      .rx_buf = data,
      .rx_len = len,
      .restart = 1,
      .callback = NULL,
  };
  int result = MXC_I2C_MasterTransaction(&req);
  return result;
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
  // uint32_t t_start = MXC_TMR_GetCount(MXC_TMR1);
  interrupt = 1; // Set interrupt flag to indicate that a sample was discarded

  if (samples_discarded == 7)
  {
    sample_ready = 1;
    samples_discarded = 0;
  }
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
  if (MXC_I2C_Init(I2C_MASTER, 1, 0) != E_NO_ERROR)
  {
    printf("I2C init failed\n");
    return -1;
  }
  MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
  uint8_t whoami = 0;
  if (i2c_read(WHO_AM_I_REG, &whoami, 1) != 0 || whoami != 0x6A)
  {
    printf("WHO_AM_I: 0x%02X (Expected 0x6A)\nDevice not recognized.\n", whoami);
    return -1;
  }
  printf("WHO_AM_I: 0x%02X (Expected 0x6A)\n", whoami);
  // Soft reset
  i2c_write(CTRL3_C, 0x01);
  MXC_Delay(MXC_DELAY_MSEC(100));
  i2c_write(CTRL3_C, 0x44); // BDU=1, IF_INC=1
  // Enable accelerometer and gyro
  i2c_write(CTRL1_XL, 0x40);      // Accel ODR = 104Hz
  i2c_write(CTRL2_G, 0x48);       // Gyro ODR = 104Hz
  i2c_write(CTRL10_C, 0x38);      // Enable XL/G axes
  i2c_write(MASTER_CONFIG, 0x00); // Disable embedded functions
  printf("Sensor fully configured. Starting loop...\n");
  MXC_Delay(MXC_DELAY_MSEC(100)); // Allow sensor to settle
  printf("Calibrating gyro... keep device still.\n");
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++)
  {
    uint8_t raw[6];
    for (int j = 0; j < 6; ++j)
      i2c_read(OUTX_L_G + j, &raw[j], 1);
    int16_t gx = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t gy = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t gz = (int16_t)(raw[5] << 8 | raw[4]);

    gx_offset += gx * 8.75f / 1000.0f;
    gy_offset += gy * 8.75f / 1000.0f;
    gz_offset += gz * 8.75f / 1000.0f;

    MXC_Delay(MXC_DELAY_MSEC(50));
  }
  gx_offset /= GYRO_CALIB_SAMPLES;
  gy_offset /= GYRO_CALIB_SAMPLES;
  gz_offset /= GYRO_CALIB_SAMPLES;
  printf("Gyro bias: X=%.2f, Y=%.2f, Z=%.2f\n", gx_offset, gy_offset, gz_offset);
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

  while (1)
  {
    WsfTimerSleepUpdate();
    // uint32_t start = MXC_TMR_GetCount(MXC_TMR1);

    wsfOsDispatcher();
    if (interrupt)
    {
      if (sample_ready)
      {
        sample_ready = 0;

        // Read accel
        uint8_t acc_raw[6];
        for (int i = 0; i < 6; ++i)
          i2c_read(OUTX_L_XL + i, &acc_raw[i], 1);
        int16_t ax = (int16_t)(acc_raw[1] << 8 | acc_raw[0]);
        int16_t ay = (int16_t)(acc_raw[3] << 8 | acc_raw[2]);
        int16_t az = (int16_t)(acc_raw[5] << 8 | acc_raw[4]);

        // Read gyro
        uint8_t gyr_raw[6];
        for (int i = 0; i < 6; ++i)
          i2c_read(OUTX_L_G + i, &gyr_raw[i], 1);
        int16_t gx = (int16_t)(gyr_raw[1] << 8 | gyr_raw[0]);
        int16_t gy = (int16_t)(gyr_raw[3] << 8 | gyr_raw[2]);
        int16_t gz = (int16_t)(gyr_raw[5] << 8 | gyr_raw[4]);

        // Convert accel
        float ax_g = ax * 0.061f / 1000.0f;
        float ay_g = ay * 0.061f / 1000.0f;
        float az_g = az * 0.061f / 1000.0f;

        // Convert gyro (raw) and subtract bias
        float gx_dps = gx * 8.75f / 1000.0f - gx_offset;
        float gy_dps = gy * 8.75f / 1000.0f - gy_offset;
        float gz_dps = gz * 8.75f / 1000.0f - gz_offset;

        imu_data_t data = {
            .ax = ax_g,
            .ay = ay_g,
            .az = az_g,
            .gx = gx_dps,
            .gy = gy_dps,
            .gz = gz_dps,
        };
        spiBurst(&data);

        current_freq = !current_freq;
        setFreq(current_freq);
        interrupt = 0;
        // uint32_t end = MXC_TMR_GetCount(MXC_TMR1);
        // uint32_t delta = end - start;
        // printf("time for burst: %lu cycles\n", delta);
      }
      else
      {
        sample_index++;
        samples_discarded++;
        spiBurstnoPrint();
        // uint32_t end = MXC_TMR_GetCount(MXC_TMR1);
        // uint32_t delta = end - start;
        interrupt = 0;
        // printf("time for discarded burst: %lu cycles\n", delta);
      }
      // uint32_t now = MXC_TMR_GetCount(MXC_TMR1);
      // uint32_t delta = now - last_call;
      // printf("Time since last burst: %lu cycles\n", delta);
      // last_call = now;

      wsfOsDispatcher();

      if (!WsfOsActive())
      {
        WsfTimerSleep();
      }
    }
  }
  printf("error count = %d\n", errCnt);

  shutdownSPI();
  umount();

  printf("Finished\n");

  return E_NO_ERROR;
}