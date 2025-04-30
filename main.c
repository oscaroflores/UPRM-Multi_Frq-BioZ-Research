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
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
extern int count;
volatile bool fifoNeedsService = false; // Global
extern uint32_t sample_interval_us;
volatile bool buttonPressed = false; // Global flag
void buttonISR(void *unused) {
  // 1. Clear the GPIO interrupt flag
  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_2);

  if(regRead(0x20)==0x00){
    regWrite(0x20,0xBF);
  }
  else{
    regWrite(0x20,0x00);
  }
}
void setupButtonInterrupt(){
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
void sensorISR(void *unused) {
  // 1. Clear the GPIO interrupt flag
  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_25);

  // 2. Call your main AFE interrupt handler
  
  spiBurst();
}
void setupMax30009Interrupt(void) {
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

int main(void) {
  int err;

  printf("START\n");

  waitCardInserted();
  createNextBiozLogFile();

  while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {
  }

  if ((err = MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), user_commands,
                          num_user_commands)) != E_NO_ERROR) {
    return err;
  }

  initSPI();       // Setup SPI
  init();          // Setup sensor (MAX30009)
  SFBIAsettings(); // Set up correct sensor registers for SFBIA
  setupButtonInterrupt(NULL);
  setupMax30009Interrupt(); // <<< YOU ADD THIS FUNCTION CALL <<<

  // <<< NEW: Setup GPIO interrupt for P0_25 here >>>

  // Setup timer (you already have this, no changes)
  mxc_tmr_cfg_t tmr_cfg;
  tmr_cfg.pres = TMR_PRES_1;
  tmr_cfg.mode = TMR_MODE_CONTINUOUS;
  tmr_cfg.cmp_cnt = 0xFFFFFFFF;
  tmr_cfg.pol = 0;
  tmr_cfg.bitMode = TMR_BIT_MODE_32;
  tmr_cfg.clock = MXC_TMR_APB_CLK;

  MXC_TMR_Init(MXC_TMR0, &tmr_cfg, false);
  MXC_TMR_Start(MXC_TMR0);
  sample_interval_us = getSampleIntervalUS();

  // Main loop
  while (1) {

    regRead(0x00);
  }

  printf("error count = %d\n", errCnt);

  shutdownSPI();
  umount();

  printf("Finished\n");

  return E_NO_ERROR;
}
