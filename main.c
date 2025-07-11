/*
This is the main file for communication done between the MAX32655 and the
MAX30009 to measure GSR


The current setup set out good frequencies for GSR, but there are functions to
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
/***** Definitions *****/
#define SPI_SPEED 1000000 // Bit Rate

// Board Selection
#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn

/***** Globals *****/
bool current_freq = 0;
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
extern uint32_t sample_interval_us;
extern sample_index;
int samples_discarded;

void buttonISR(void *unused)
{
  // 1. Clear the GPIO interrupt flag
  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_2);

  if (regRead(0x20) == 0x00)
  {
    changeReg(0x20, 0x7, 2, 3);
    sample_index = 0;
  }
  else
  {
    changeReg(0x20, 0x0, 2, 3);
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

  if (samples_discarded < 7)
  {
    samples_discarded++;
    sample_index++;
    spiBurstnoPrint(); // Read the FIFO, but don't print anything
    // printf("discarded = %d\n", samples_discarded);
  }
  else
  {

    // uint32_t t_start = MXC_TMR_GetCount(MXC_TMR1);
    double freqLogged = getBiozFreq();
    spiBurst(freqLogged); // Time this

    // Alternate frequency AFTER processing the current burst
    current_freq = !current_freq;
    setFreq(current_freq);

    // uint32_t t_end = MXC_TMR_GetCount(MXC_TMR1);
    // uint32_t delta = t_end - t_start;

    // printf("spiBurst took %lu cycles\n", delta);
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

  int err;

  printf("START\n");

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
  timer1_cfg.clock = MXC_TMR_APB_CLK;
  timer1_cfg.cmp_cnt = 0xFFFFFFFF;
  timer1_cfg.pol = 0;

  MXC_TMR_Init(MXC_TMR1, &timer1_cfg, false);
  MXC_TMR_Start(MXC_TMR1);

  // Main loop
  while (1)
  {
  }

  printf("error count = %d\n", errCnt);

  shutdownSPI();
  umount();

  printf("Finished\n");

  return E_NO_ERROR;
}