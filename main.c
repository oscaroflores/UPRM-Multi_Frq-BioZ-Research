/*
This is the main file for communication done between the MAX32655 and the
MAX30009 to measure GSR


The current setup set out good frequencies for GSR, but there are functions to
easily change the settings to measure at different rates or to measure different
vital signs depending on necessities






*/
#include "MAX30009.h"
#include "MAX32655.h"
#include "board.h"
#include "cli.h"
#include "dma.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "sdhc.h"
#include "spi.h"
#include "uart.h"
#include "user-cli.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sdhc.h"
#include "bioZ.h"
#include "spiFunctions.h"

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

int main(void)
{
  int err;

  printf("START\n");

  waitCardInserted();

  // Create file that will store spi burst data
  createNextBiozLogFile();
  printf("next bioz log: %s\n", new_log_file);

  while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)))
  {
  }

  // Init CLI
  if ((err = MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), user_commands, num_user_commands)) !=
      E_NO_ERROR)
  {
    return err;
  }

  // Collection of reads or writes including initialisation
  initSPI(); // begin SPI communication
  init(); // initialise the MAX30009
  SFBIAsettings(); // Put in correct setting for SFBIA communication
  setMode(0);
  spiBurst(); // Begin reading from the FIFO

  printf("error count = %d\n", errCnt);

  shutdownSPI();
  umount();

  printf("Finished\n");

  return E_NO_ERROR;
}
