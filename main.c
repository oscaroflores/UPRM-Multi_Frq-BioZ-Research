#include "MAX30009.h"
#include "MAX32655.h"
#include "board.h"
#include "dma.h"
#include "gpio.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "spi.h"
#include "uart.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bioZ.h"
#include "spiFunctions.h"

/***** Definitions *****/
#define SPI_SPEED 1000000 // Bit Rate

#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn

/***** Globals *****/
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
extern int count;

volatile bool measuring = false; // Toggle flag via interrupt
volatile bool state_changed = false;
bool sensor_initialized = false; // Tracks if sensor/SPI are active

/***** Function Prototypes *****/
void setupButtonInterrupt(void);

/***** Interrupt Handler for Button *****/
void GPIO0_IRQHandler(void) {
  if (MXC_GPIO_GetFlags(MXC_GPIO0) & MXC_GPIO_PIN_2) {
    MXC_GPIO_ClearFlags(MXC_GPIO0, MXC_GPIO_PIN_2);
    measuring = !measuring;

    if (!measuring) {
      // Reset the count when measurement is stopped
      changeReg(0x20, 0, 2, 3);
      regWrite(0x20, regRead(0x20) & 0xF8);
      changeReg(0x17, 0, 0, 1);
    }
    if (measuring) {
      // Reset the count when measurement is started
      // Attempt to bring the sensor out of shutdown by performing a soft
      // reset. For instance, clear the reset/shutdown bits in the system
      // configuration register.
      // Reinitialize the SPI interface and sensor registers
      initSPI();
      init(); // Your custom sensor initialization
      regWrite(SYSTEM_CONFIGURATION_1_REGISTER, 0x00);
      MXC_Delay(10000); // Delay to allow the sensor to power up

      // Re-enable BioZ channels by setting the lower 3 bits of register 0x20.
      regWrite(0x20, (regRead(0x20) & 0xF8) | 0x07);

      // Re-enable the PLL by setting bit 0 in register 0x17.
      changeReg(0x17, 1, 0, 1);

      // Restore other settings as needed.
      SFBIAsettings();
      setMode(0);

      sensor_initialized = true;
    }
    state_changed = true;
    printf("Measurement %s\n", measuring ? "started" : "stopped");
  }
}

/***** GPIO Button Setup *****/
void setupButtonInterrupt(void) {
  mxc_gpio_cfg_t button_cfg = {
      .port = MXC_GPIO0,
      .mask = MXC_GPIO_PIN_2,
      .func = MXC_GPIO_FUNC_IN,
      .pad = MXC_GPIO_PAD_PULL_UP,
      .vssel = MXC_GPIO_VSSEL_VDDIO,
  };

  MXC_GPIO_Config(&button_cfg);

  // Register callback and enable interrupt on falling edge
  MXC_GPIO_RegisterCallback(&button_cfg, GPIO0_IRQHandler, NULL);
  MXC_GPIO_IntConfig(&button_cfg, MXC_GPIO_INT_FALLING);

  MXC_GPIO_EnableInt(MXC_GPIO0, MXC_GPIO_PIN_2);
  NVIC_EnableIRQ(GPIO0_IRQn);
}

int main(void) {
  printf("System initializing...\n");
  // You might want to call initSPI() here if needed
  setupButtonInterrupt();
  printf("System ready. Press W2 (Port0.2) to begin or stop measurement.\n");

  bool prev_state = measuring;
  while (1) {
    if (measuring) {
      spiBurst();
      MXC_Delay(100000); // Idle delay

    } else {
      MXC_Delay(100000); // Idle delay
    }
  }
  shutdownSPI();

  return E_NO_ERROR;
}
