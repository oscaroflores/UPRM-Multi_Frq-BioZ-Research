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
#include "board.h"
#include "dma.h"
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
#include "tmr.h"

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
  /*
  Collection of reads or writes including initialisation
  */
  initSPI(); // begin SPI communication
  init();    // initialise the MAX30009
             //   GSRsettings();//put in correct setting for GSR communication
  SFBIAsettings(); // put in correct setting for SFBIA communication
  // setMode(0);
  mxc_tmr_cfg_t tmr_cfg;
  tmr_cfg.pres = TMR_PRES_1; // No prescaler (8 MHz)
  tmr_cfg.mode = TMR_MODE_CONTINUOUS;
  tmr_cfg.cmp_cnt = 0xFFFFFFFF;
  tmr_cfg.pol = 0;
  tmr_cfg.bitMode = TMR_BIT_MODE_32;
  tmr_cfg.clock = MXC_TMR_APB_CLK;

  MXC_TMR_Init(MXC_TMR0, &tmr_cfg, false); // false = don't reinit pins
  MXC_TMR_Start(MXC_TMR0);
  start_time_ms = MXC_TMR_GetCount(MXC_TMR0);
  spiBurst(); // begin reading from the FIFO

  printf("error count = %d\n", errCnt);

  shutdownSPI();

  printf("Finished\n");

  return E_NO_ERROR;
}