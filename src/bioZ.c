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

extern int current_freq_kHz;
extern uint8_t gReadBuf[100];
uint8_t adcIData[10];
uint8_t adcQData[10];
extern uint8_t IMag;
extern int count;
extern int errCnt;
// sample interval
uint32_t sample_interval_us = 0; // make accessible from main if needed
uint32_t sample_index = 0;       // Declare as global variable
// uint32_t sr_bioz ;
void GSRsettings()
{
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
void setMdiv(int val)
{
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
void SFBIAsettings()
{
  // --- Reset and bring device out of shutdown ---
  regWrite(0x20, 1 << 2); // BIOZ_BG_EN
  regWrite(0x11, 0);      // clear SHDN
  regWrite(0x17, 0);      // clear PLL_EN
  regWrite(0x1A, 0);      // clear REF_CLK_SEL
  regWrite(0x11, 1);      // issue RESET

  // --- Clear status ---
  regRead(0x00); // read status to clear

  // --- PLL, Clock and Gain Setup ---
  regWrite(0x18, 0xBB);                               // MDIV
  regWrite(0x19, 0x01);                               // PLL_LOCK_WNDW
  regWrite(0x1A, (0 << 6) | (1 << 5));                // REF_CLK_SEL = internal, CLK_FREQ_SEL = 32768Hz
  regWrite(0x17, (1 << 6) | (0 << 5) | (2 << 1) | 1); // PLL_EN, NDIV=0=>1024, KDIV=2=>4

  // --- Basic Sensor/AFE Configuration ---
  regWrite(0x10, 0x00); // BIOZ_CONFIG
  regWrite(0x12, 0x04); // CMDET_EN
  regWrite(0x13, 0x00); // No Lead-Off Comparator
  regWrite(0x14, 0x00); // Reserved
  regWrite(0x21, 0x20); // HIZ after reset
  regWrite(0x22, 0x28); // BIOZ_VDRV_MAG = 2 (2.5x), BIOZ_IDRV_RGE = 2 (64uA)
  regWrite(0x23, 0x00); // default
  regWrite(0x24, 0x33); // Amp Biasing
  regWrite(0x25, 0xCA); // AMP_RGE and bandwidth
  regWrite(0x26, 0x00);
  regWrite(0x27, 0xFF);
  regWrite(0x28, 0x02); // Drive Freq Trim
  regWrite(0x41, 0x06); // BIOZ_MUX Config, MUX_EN
  regWrite(0x42, 0x01); // MUX2
  regWrite(0x43, 0xA0); // BIOZ_RBIAS and Bias Driver
  regWrite(0x44, 0xD1); // Refbuf trim
  regWrite(0x50, 0x00);
  regWrite(0x51, 0x00);
  regWrite(0x58, 0x07); // All clock gates on

  // --- Interrupt Setup ---
  regWrite(0x80, 0xA0); // Enable A_FULL_EN and FIFO_DATA_RDY_EN
  regWrite(0x81, 0x00); // Optional: disable error interrupts for now

  // --- DAC/ADC OSR Config (will be overwritten in setFreq()) ---
  regWrite(0x20, 0xB0); // DAC_OSR = 2 (128), ADC_OSR = 4 (128), BG_EN, Q_EN, I_EN = 0

  // --- Note: Must call setFreq() after this to finalize DAC/ADC and KDIV settings
}

uint32_t getRefClkHz()
{

  uint8_t ref_clk_sel = regRead(0x1A) & 0b01000000;  // Bit 6
  uint8_t clk_freq_sel = regRead(0x1A) & 0b00100000; // Bit 5
  // printf("ref_clk_sel: %d\t", ref_clk_sel);
  // printf("clk_freq_sel: %d\n", clk_freq_sel);
  if (ref_clk_sel == 0)
  {
    // Internal oscillator
    return (clk_freq_sel == 0) ? 32000 : 32768;
  }
  else
  {
    // External oscillator
    return (clk_freq_sel == 0) ? 32000 : 32768;
  }
}
double sr_bioz;

double getSampleInterval()
{
  uint32_t ref_clk = getRefClkHz();

  int mdiv_high = (regRead(0x17) >> 6) & 0x03;
  int mdiv_low = regRead(0x18);
  int mdiv = (mdiv_high << 8) | mdiv_low;
  // printf("M Divider: %d\n", mdiv);

  double pll_clk = (double)ref_clk * (mdiv + 1);
  // printf("PLL Clock: %.2f Hz\n", pll_clk);
  uint8_t ndiv_raw = (regRead(0x17) >> 5) & 0x01;
  double ndiv = (ndiv_raw == 0) ? 512.0 : 1024.0;
  // printf("N Divider: %.0f\n", ndiv);

  uint8_t adc_osr_raw = (regRead(0x20) >> 3) & 0x07;
  uint16_t adc_osr_table[] = {8, 16, 32, 64, 128, 256, 512, 1024};
  double bioz_adc_osr = adc_osr_table[adc_osr_raw];
  // printf("BioZ ADC OSR: %.0f\n", bioz_adc_osr);

  sr_bioz = pll_clk / (ndiv * bioz_adc_osr);
  // printf("SR: %.4f\n", sr_bioz); // Prints with more decimal places

  if (sr_bioz == 0.0)
    return 0.0;
  return 1 / sr_bioz;
}

int getDACOSR()
{
  uint8_t val = (regRead(0x20) & 0b11000000) >> 6;
  switch (val)
  {
  case 0:
    return 32;
  case 1:
    return 64;
  case 2:
    return 128;
  case 3:
    return 256;
  default:
    return 32; // fallback
  }
}
int getKDiv()
{
  /*
  This function returns the K divider value
  */

  uint8_t k_div = (regRead(0x17) & 0b00011110) >> 1; // Bits 4:1
  uint16_t k_div_table[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 8192, 8192};

  if (k_div < 14)
  {
    return k_div_table[k_div];
  }
  else
  {
    return 8192;
  }
}
void setFreq(int freq)
{
  /*
  Configures frequency-specific settings for BioZ signal generation
  Supports: 5 (kHz), 150 (kHz)
  */
  switch (freq)
  {
  case 5:
    changeReg(0x17, 1, 5, 1); // NDIV = 1 (1024)
    changeReg(0x17, 4, 4, 4); // k_div = 16
    changeReg(0x20, 3, 7, 2); // DAC_OSR = 256
    setMdiv(625);
    break;

  case 150:
    changeReg(0x17, 1, 5, 1); // NDIV = 1 (1024)
    changeReg(0x17, 0, 4, 4); // k_div = 1
    changeReg(0x20, 2, 7, 2); // sets bits 7:6 = 10
    setMdiv(586);
    break;

  default:
    printf("Invalid frequency: %d kHz\n", freq);
    break;
  }
}

int calcBioZ(uint8_t buf[], uint32_t timestamp_us_unused)
{
  /*
  This function uses the readings from the FIFO register
  */

  // sample_index persists as a global variable
  uint8_t x1[3], x2[3];
  int i, err = 0;

  for (i = 0; i < 3; i++)
  {
    x1[i] = buf[i];
    x2[i] = buf[i + 3];
  }

  uint8_t a = x1[0] & 0xF0;
  uint8_t b = x2[0] & 0xF0;

  /***************************************************
  Find which array is Quadrature phase and which is In phase
  ********************************************************/
  if ((a == 0x10) && (b == 0x20))
  {
    adcIData[0] = (x1[0] & 0x0F);
    adcIData[1] = x1[1];
    adcIData[2] = x1[2];

    adcQData[0] = (x2[0] & 0x0F);
    adcQData[1] = x2[1];
    adcQData[2] = x2[2];
  }
  else if ((a == 0x20) && (b == (0x10)))
  {
    adcIData[0] = (x2[0] & 0x0F);
    adcIData[1] = x2[1];
    adcIData[2] = x2[2];

    adcQData[0] = (x1[0] & 0x0F);
    adcQData[1] = x1[1];
    adcQData[2] = x1[2];
  }
  else if (x1[0] == 0xFF && x2[0] == 0xFF && x1[1] == 0xFF && x2[1] == 0xFF &&
           x1[2] == 0xFF && x2[2] == 0xFF)
  {
    printf("Invalid Data\n");
    return 1;
  }
  else if (x1[0] == 0xFF && x2[0] == 0xFF && x1[1] == 0xFF && x2[1] == 0xFF &&
           x1[2] == 0xFE && x2[2] == 0xFF)
  {
    printf("Marker\n");
    return 2;
  }
  else
  {
    errCnt++;
    return 3;
  }

  double I, Q, temp;
  uint32_t L, d;

  uint32_t adcI = adcIData[2] + (adcIData[1] << 8) + (adcIData[0] << 16);
  uint32_t adcQ = adcQData[2] + (adcQData[1] << 8) + (adcQData[0] << 16);

  if (adcI >> 19 == 1)
  {
    L = (adcI & 0x7FFFF);
    d = 0x80000;
    I = L;
    temp = d;
    I = I - temp;
    adcI = adcI >> 1;
  }
  else
  {
    I = adcI;
  }

  if (adcQ >> 19 == 1)
  {
    L = (adcQ & 0x7FFFF);
    d = 0x80000;
    Q = L;
    temp = d;
    Q = Q - temp;
    adcQ = adcQ >> 1;
  }
  else
  {
    Q = adcQ;
  }

  // --- Timestamp using sample index and sr_bioz ---
  uint32_t timestamp = ((uint32_t)(sample_index * (1.0 / sr_bioz) * 1e3)); // in miliseconds
  sample_index++;                                                          // Increment sample index for next sample timestamp

  // Calculate M divider value from registers 0x17 and 0x18
  // int mdiv_high = (regRead(0x17) >> 6) & 0x03;
  // int mdiv_low = regRead(0x18);
  // int M = (mdiv_high << 8) | mdiv_low;

  // double PLL_CLK = getRefClkHz() * (M + 1);            // Calculate PLL clock frequency
  // double F_BIOZ = PLL_CLK / (getKDiv() * getDACOSR()); // Calculate BioZ frequency

  // Debugging prints

  // printf("M Divider: %d\n", M);
  // printf("Ref Clock: %d Hz\n", getRefClkHz());
  // printf("DAC OSR: %d\n", getDACOSR());
  // printf("K Divider: %d\n", getKDiv());
  // printf("BioZ Frequency: %.2f Hz\n", F_BIOZ);
  // printf("t = %lu ms\tFreq = %d kHz\tQ = %.2f\tI = %.2f\t SR: %.4f\t OVF: %d\n", timestamp, current_freq_kHz, Q, I, sr_bioz, regRead(0x0A) & 0x0F);
  // printf("overflow count = %d\n", regRead(0x0A) & 0x0F); // Read overflow count from register 0x1B
  // printf("SR: %.4f\n", sr_bioz);
  // -- Print the results --
  // printf("%lu\t", timestamp);
  // printf("%.1f\t", Q);
  // printf("%.1f\t", I);
  printf("%d\n", current_freq_kHz);
  // printf("OVF: %d\n", regRead(0x0A) & 0x80); // Read overflow count from register 0x0A

  // SD card upload
  // char log_entry[128];
  // int log_len = snprintf(log_entry, sizeof(log_entry), "%lu,%d,%d,%d\n", timestamp, Q, I, current_freq_kHz);

  // if (log_len < 0 || log_len >= sizeof(log_entry))
  // {
  //   printf("Error formatting log entry.\n");
  //   return -1;
  // }

  // setMessage(log_entry);
  // appendFile(new_log_file, log_len);

  return err;
}
