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
#include "spiFunctions.h"
#include "uart.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "spiFunctions.h"
extern uint8_t gReadBuf[100];
void init(void) // call this on power up
{

  // the register sequence in this section is required for RESET
  regWrite(0x20, 1 << 2); // BIOZ_BG_EN
  regWrite(0x11, 0);      // clear SHDN
  regWrite(0x17, 0);      // clear PLL_EN
  regWrite(0x1a, 0);      // clear REF_CLK_SEL
  // sleep(1000);
  regWrite(0x11, 1); // RESET

  // regRead(0x00, NUM_STATUS_REGS);	// read and clear all status registers
  regRead(0x00);
  // regWrite(0x0d, AFE_FIFO_SIZE-NUM_SAMPLES_PER_INT);	// FIFO_A_FULL; assert
  // A_FULL on NUM_SAMPLES_PER_INT samples
  regWrite(0x80, 0x80); // A_FULL_EN; enable interrupt pin on A_FULL assertion
  regWrite(0x18, 0xbb); // MDIV
  regWrite(0x19, 0x01); // PLL_LOCK_WNDW
  regWrite(0x1a, (0 << 6) | (1 << 5)); // REF_CLK_SEL | CLK_FREQ_SEL
  regWrite(0x17,
           (1 << 6) | (0 << 5) | (2 << 1) | 1); // MDIV | NDIV | KDIV | PLL_EN
  regWrite(0x22, (3 << 4) | (2 << 2)); // BIOZ_VDRV_MAG | BIOZ_IDRV_RGE: 64uA
  regWrite(0x25, (3 << 2) | 3);        // BIOZ_AMP_RGE | BIOZ_AMP_BW
  regWrite(0x41, 1 << 1);              // MUX_EN
  // ensure to enable the MCU's interrupt pin before enabling BIOZ
  regWrite(0x20, (3 << 6) | (4 << 3) | (1 << 2) | (1 << 1) |
                     1); // BIOZ_DAC_OSR | BIOZ_ADC_OSR | BIOZ_BG_EN | BIOZ_Q_EN
                         // | BIOZ_I_EN: ADC 222Sps, DAC 14.208kHz
}
void MAX30009onAfeInt(void) {
  //   printf("MAX30009onAfeInt\n");

  regRead(0x00); // Read and clear status registers

  spiBurst();                  // ✅ read FIFO properly
  if (!(gReadBuf[0] & 0x20)) { // Check DATA_RDY bit
    return;                    // No new data
  }

  // Now gReadBuf should already have the FIFO data ready
}
