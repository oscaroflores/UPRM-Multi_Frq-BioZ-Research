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
#include "dma.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "sdhc.h"
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

// Board Selection
#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn

/***** Globals *****/
uint8_t gReadBuf[100];
uint8_t gHold[100];
int errCnt;
extern int count;

int main(void) {

  printf("START\n");

  // Collection of reads or writes including initialisation
  initSPI(); // begin SPI communication

  init(); // initialise the MAX30009

  SFBIAsettings(); // Put in correct setting for SFBIA communication
  setMode(0);

  spiBurst(); // Begin reading from the FIFO

  
  printf("error count = %d\n", errCnt);
  
  shutdownSPI();
  
  waitCardInserted(); // Wait for SD Card to be inserted
  generateMessage(1024); // Message to be written to sd card
  createFile("Test.txt", 1024); // Create and write message to file

  printf("Finished\n");

  return E_NO_ERROR;
}
