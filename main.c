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

volatile bool buttonPressed = false; // Global flag
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
void GPIO_InterruptSelfTest(void) {
  printf("Starting GPIO Interrupt Self-Test...\n");

  // 1. Check if P0.25 is input
  uint32_t dir = MXC_GPIO0->outen;
  if (dir & MXC_GPIO_PIN_25) {
    printf("[FAIL] P0.25 is configured as OUTPUT. Should be INPUT!\n");
  } else {
    printf("[PASS] P0.25 is configured as INPUT.\n");
  }

  // 2. Check if interrupt enabled for P0.25
  uint32_t int_en = MXC_GPIO0->inten;
  if (int_en & MXC_GPIO_PIN_25) {
    printf("[PASS] P0.25 interrupt is ENABLED.\n");
  } else {
    printf("[FAIL] P0.25 interrupt is NOT enabled!\n");
  }

  // 3. Check if edge config is FALLING
  uint32_t int_mode = MXC_GPIO0->intmode;
  uint32_t int_pol = MXC_GPIO0->intpol;
  if ((int_mode & MXC_GPIO_PIN_25) && !(int_pol & MXC_GPIO_PIN_25)) {
    printf("[PASS] P0.25 interrupt configured for FALLING edge.\n");
  } else {
    printf("[FAIL] P0.25 not correctly set for FALLING edge!\n");
  }

  // 4. Check if NVIC IRQ for GPIO0 is enabled
  if (NVIC_GetEnableIRQ(MXC_GPIO_GET_IRQ(0))) {
    printf("[PASS] NVIC interrupt for GPIO0 block is ENABLED.\n");
  } else {
    printf("[FAIL] NVIC interrupt for GPIO0 block is NOT enabled!\n");
  }

  // 5. Check if INTB is physically low right now
  if (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_25) == 0) {
    printf("[INFO] INTB (P0.25) is currently LOW — sensor is asserting "
           "interrupt.\n");
  } else {
    printf("[INFO] INTB (P0.25) is currently HIGH — sensor idle.\n");
  }

  printf("GPIO Interrupt Self-Test completed.\n");
}

void buttonISR(void *unused) {
  // Clear the interrupt flag
  MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_2);

  // Set our flag
  buttonPressed = true;
}

void setupButtonInterrupt() {
  mxc_gpio_cfg_t sensorPin = {
      .port = MXC_GPIO0,
      .mask = MXC_GPIO_PIN_25,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP, // Enable pull-up resistor because button
                                   // pulls low
      .vssel = MXC_GPIO_VSSEL_VDDIO};

  MXC_GPIO_Config(&sensorPin);

  // Register the ISR
  MXC_GPIO_RegisterCallback(&sensorPin, sensorISR, NULL);

  // Configure interrupt on falling edge
  MXC_GPIO_IntConfig(&sensorPin, MXC_GPIO_INT_FALLING);

  // Enable GPIO interrupt for this pin
  MXC_GPIO_EnableInt(sensorPin.port, sensorPin.mask);

  // Enable the GPIO0 block interrupt in NVIC
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(0));
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

  setupMax30009Interrupt(); // <<< YOU ADD THIS FUNCTION CALL <<<

  GPIO_InterruptSelfTest();
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
  start_time_ms = MXC_TMR_GetCount(MXC_TMR0);

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
