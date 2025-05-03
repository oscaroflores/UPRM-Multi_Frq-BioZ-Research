#include "MAX30009.h"
#include "MAX32655.h"
#include "board.h"
#include "dma.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "sdhc.h"
#include "spi.h"
#include "spiFunctions.h"
#include "tmr.h"
#include "uart.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

uint32_t start_time_ms;

extern uint8_t gReadBuf[100];
uint8_t adcIData[10];
uint8_t adcQData[10];
extern uint8_t IMag;
extern int count;
extern int errCnt;
// sample interval
uint32_t sample_interval_us = 0; // make accessible from main if needed
// uint32_t sr_bioz ;
void GSRsettings() {
  /*

  These are the necessary settings to measure GSR


  */

  regWrite(0x10, 0x00);
  regWrite(0x12, 0x04);
  regWrite(0x13, 0x00);
  regWrite(0x14, 0x00);
  regWrite(0x17, 0xF9);
  regWrite(0x18, 0x1F);
  regWrite(0x19, 0x01);
  regWrite(0x1A, 0x20);
  regWrite(0x20, 0xFF);
  regWrite(0x21, 0x00);
  regWrite(0x22, 0x14);
  regWrite(0x23, 0x00);
  regWrite(0x24, 0xF0);
  regWrite(0x25, 0x40);
  regWrite(0x26, 0x00);
  regWrite(0x27, 0xFF);
  regWrite(0x28, 0x02);
  regWrite(0x41, 0x06);
  regWrite(0x42, 0x01);
  regWrite(0x43, 0xA0);
  regWrite(0x50, 0x00);
  regWrite(0x51, 0x00);
  regWrite(0x58, 0x07);
  regWrite(0x80, 0x80);
  regWrite(0x81, 0x00);
}
void setMdiv(int val) {
  /*
  This function is specifically to change the M divider value
  as it spans over two seperate registers


  */
  uint32_t V = val;

  uint8_t a = V >> 8;

  uint8_t b = V & 0xFF;

  changeReg(0x17, a, 7, 2); // step 3:
  changeReg(0x18, b, 7, 8); // MDIV
}
void SFBIAsettings() {
  /*
  These are the necessary settings to measure SFBIA


  */
  regWrite(0x10, 0x00);
  // regWrite(0x11, 0x02);
  regWrite(0x12, 0x04);
  regWrite(0x13, 0x00);
  regWrite(0x14, 0x00);
  regWrite(0x17, 0x81);
  regWrite(0x18, 0x49);
  regWrite(0x19, 0x01);
  regWrite(0x1A, 0x20);
  regWrite(0x20, 0x00);
  regWrite(0x21, 0x20);
  regWrite(0x22, 0x28);
  regWrite(0x23, 0x00);
  regWrite(0x24, 0x33);
  regWrite(0x25, 0xCA);
  regWrite(0x26, 0x00);
  regWrite(0x27, 0xFF);
  regWrite(0x28, 0x02);
  regWrite(0x41, 0x06);
  regWrite(0x42, 0x01);
  regWrite(0x43, 0xA0);
  regWrite(0x44, 0xD1);
  regWrite(0x50, 0x00);
  regWrite(0x51, 0x00);
  regWrite(0x58, 0x07);
  regWrite(0x80, 0xA0);
  regWrite(0x81, 0x00);
  changeReg(0x17, 1, 5, 1); // NDIV = 1 (1024)
  changeReg(0x20, 6, 4, 3); // ADC_OSR index 6 → 512
  setMdiv(586);             // MDIV = 586
}

uint32_t getRefClkHz() {

  uint8_t ref_clk_sel = regRead(0x1A) & 0b01000000;  // Bit 6
  uint8_t clk_freq_sel = regRead(0x1A) & 0b00100000; // Bit 5
  // printf("ref_clk_sel: %d\t", ref_clk_sel);
  // printf("clk_freq_sel: %d\n", clk_freq_sel);
  if (ref_clk_sel == 0) {
    // Internal oscillator
    return (clk_freq_sel == 0) ? 32000 : 32768;
  } else {
    // External oscillator
    return (clk_freq_sel == 0) ? 32000 : 32768;
  }
}
double sr_bioz;

double getSampleInterval() {
  uint32_t ref_clk = getRefClkHz();

  uint8_t mdiv_high = (regRead(0x17) >> 6) & 0x03;
  uint8_t mdiv_low = regRead(0x18);
  uint16_t mdiv = (mdiv_high << 8) | mdiv_low;

  double pll_clk = (double)ref_clk * (mdiv + 1);

  uint8_t ndiv_raw = (regRead(0x17) >> 5) & 0x01;
  double ndiv = (ndiv_raw == 0) ? 512.0 : 1024.0;

  uint8_t adc_osr_raw = (regRead(0x20) >> 2) & 0x07;
  uint16_t adc_osr_table[] = {8, 16, 32, 64, 128, 256, 512, 1024};
  double bioz_adc_osr = adc_osr_table[adc_osr_raw];

  sr_bioz = pll_clk / (ndiv * bioz_adc_osr);
  // printf("SR: %.4f\n", sr_bioz);  // Prints with more decimal places

  if (sr_bioz == 0.0)
    return 0.0;
  return 1 / sr_bioz;
}



int calcBioZ(uint8_t buf[], uint32_t timestamp_us_unused) {
  /*
  This function uses the readings from the FIFO register
  */

  static uint32_t sample_index = 0; // persists across calls

  uint8_t x1[3], x2[3];
  int i, err = 0;

  for (i = 0; i < 3; i++) {
    x1[i] = buf[i];
    x2[i] = buf[i + 3];
  }

  uint8_t a = x1[0] & 0xF0;
  uint8_t b = x2[0] & 0xF0;

  /***************************************************
  Find which array is Quadrature phase and which is In phase
  ********************************************************/
  if ((a == 0x10) && (b == 0x20)) {
    adcIData[0] = (x1[0] & 0x0F);
    adcIData[1] = x1[1];
    adcIData[2] = x1[2];

    adcQData[0] = (x2[0] & 0x0F);
    adcQData[1] = x2[1];
    adcQData[2] = x2[2];

  } else if ((a == 0x20) && (b == (0x10))) {
    adcIData[0] = (x2[0] & 0x0F);
    adcIData[1] = x2[1];
    adcIData[2] = x2[2];

    adcQData[0] = (x1[0] & 0x0F);
    adcQData[1] = x1[1];
    adcQData[2] = x1[2];

  } else if (x1[0] == 0xFF && x2[0] == 0xFF && x1[1] == 0xFF && x2[1] == 0xFF &&
             x1[2] == 0xFF && x2[2] == 0xFF) {
    printf("Invalid Data\n");
    return 1;
  } else if (x1[0] == 0xFF && x2[0] == 0xFF && x1[1] == 0xFF && x2[1] == 0xFF &&
             x1[2] == 0xFE && x2[2] == 0xFF) {
    printf("Marker\n");
    return 2;
  } else {
    errCnt++;
    return 3;
  }

  double I, Q, temp;
  uint32_t L, d;

  uint32_t adcI = adcIData[2] + (adcIData[1] << 8) + (adcIData[0] << 16);
  uint32_t adcQ = adcQData[2] + (adcQData[1] << 8) + (adcQData[0] << 16);

  if (adcI >> 19 == 1) {
    L = (adcI & 0x7FFFF);
    d = 0x80000;
    I = L;
    temp = d;
    I = I - temp;
    adcI = adcI >> 1;
  } else {
    I = adcI;
  }

    if (adcQ >> 19 == 1) {
      L = (adcQ & 0x7FFFF);
      d = 0x80000;
      Q = L;
      temp = d;
      Q = Q - temp;
      adcQ = adcQ >> 1;
    } else {
    Q = adcQ;
  }

  // --- Timestamp using sample index and sr_bioz ---
  uint32_t timestamp = ((uint32_t)(sample_index * (1.0 / sr_bioz) * 1e3 * 2)); // in miliseconds
  sample_index++; // Increment sample index for next sample timestamp

  // -- Print the results --
  printf("%lu\t", timestamp);
  printf("%f\t", Q);
  printf("%f\n", I);


  // SD card upload
  char log_entry[128];
  int log_len = snprintf(log_entry, sizeof(log_entry), " %lu %f %f  \n", timestamp, Q, I);

  if (log_len < 0 || log_len >= sizeof(log_entry)) {
    printf("Error formatting log entry.\n");
    return -1;
  }

  setMessage(log_entry);
  appendFile(new_log_file, log_len);

  return err;
}
