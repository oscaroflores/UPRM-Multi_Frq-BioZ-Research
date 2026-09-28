/*
 * MAX30009 configuration and BioZ decoding for multi-frequency BIA.
 *
 * Register values are carried over verbatim from the full firmware so this spike
 * measures the same thing; only the destination changed, from BLE + SD card to
 * the console.
 *
 * Every function here performs SPI register access, so none of them may be
 * called from interrupt context. See the note at the top of spiFunctions.c.
 */

#include "bioZ.h"

#include "MAX30009.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "spiFunctions.h"
#include <math.h>
#include <stdio.h>

/***** Globals *****/

double sr_bioz;
static double bioz_adc_osr;
static double ndiv;
static double bioz_ohm_coeff = 1.0;
static uint32_t sample_index;

/* Cached so the decode path does no register reads: getBiozFreq() alone costs
 * four, for a value that only changes when the tone does. */
static double bioz_freq_cached;

/* Settling policy state. */
static uint8_t cur_tone;
static uint16_t visit_seen;
static uint16_t visit_kept;
static bool tone_just_switched;
static uint32_t decoded_count;
static uint32_t kept_count;

/***** Configuration *****/

void setMdiv(int val)
{
  uint32_t V = (uint32_t)val;

  changeReg(0x17, (uint8_t)(V >> 8), 7, 2);
  changeReg(0x18, (uint8_t)(V & 0xFF), 7, 8);
}

void BIAsettings(void)
{
  /* --- Reset --- */
  regWrite(0x20, 1 << 2); /* BIOZ_BG_EN */
  regWrite(0x11, 0);      /* clear SHDN */
  regWrite(0x17, 0);      /* clear PLL_EN */
  regWrite(0x1A, 0);      /* clear REF_CLK_SEL */
  regWrite(0x11, 1);      /* RESET */

  /*
   * The datasheet gives the bandgap up to 6 ms to settle and wants it up before
   * PLL_EN is set; a reset also needs time before the part accepts configuration.
   * The original had no delay here at all.
   */
  MXC_Delay(MXC_DELAY_MSEC(10));

  regRead(0x00); /* read status to clear */

  /* --- PLL and clock: 32768 Hz x (512 + 1) = 16,809,984 Hz --- */
  regWrite(0x18, 0xBB);
  regWrite(0x19, 0x01);
  regWrite(0x1A, (0 << 6) | (1 << 5)); /* internal osc, 32768 Hz */
  regWrite(0x17, (1 << 6) | (0 << 5) | (2 << 1) | 1);

  /* --- AFE --- */
  regWrite(0x10, 0x00); /* BIOZ_CONFIG */
  regWrite(0x12, 0x04); /* CMDET_EN */
  regWrite(0x13, 0x00); /* no lead-off comparator */
  regWrite(0x14, 0x00);
  /*
   * 0x21 is BioZ Configuration 2: DHPF[7:6], DLPF[5:3], CMP[2:1],
   * EN_BIOZ_THRESH[0] -- nothing to do with HIZ, as the original comment claimed.
   * 0x20 means DHPF bypass and DLPF = 4, so the digital low-pass filter is
   * ENABLED at 0.25 x SR_BIOZ (64 Hz here). Because the cutoff is a fraction of
   * the sample rate, this filter's settling measured in SAMPLES does not change
   * with rate -- the leading suspect for the 7-sample discard.
   */
  regWrite(0x21, 0x20);
  regWrite(0x22, 0x28); /* VDRV_MAG = 2 (250 mV), IDRV_RGE = 2 -> 45.25 uA peak */
  regWrite(0x23, 0x00);
  regWrite(0x24, 0x33); /* AHPF = 3 (1 kHz corner), GAIN = 3 (10 V/V) */
  regWrite(0x25, 0xCA); /* AMP_RGE and bandwidth */
  regWrite(0x26, 0x00);
  regWrite(0x27, 0xFF);
  regWrite(0x28, 0x12); /* drive frequency trim */
  regWrite(0x41, 0x06); /* MUX_EN + CONNECT_CAL_ONLY -- electrodes connected */
  regWrite(0x42, 0x01);
  regWrite(0x43, 0xA0); /* RBIAS and bias driver */
  /*
   * 0x44 is BIST_R_ERR: the factory-measured error of the internal BIST resistor,
   * not a "refbuf trim". Writing it destroys a calibration datum, and the ADI
   * reference never touches this register, so the write is left out here.
   */
  regWrite(0x50, 0x00);
  regWrite(0x51, 0x00);
  regWrite(0x58, 0x07); /* all clock gates on */

  /* --- Interrupts: A_FULL only ---
   * The original also set FIFO_DATA_RDY_EN (0xA0), leaving two sources driving
   * INTB with no way to tell them apart -- and FIFO_DATA_RDY asserts on every
   * sample, so it produced an interrupt per sample whatever the threshold was. */
  regWrite(0x80, 0x80); /* A_FULL_EN */
  regWrite(0x81, 0x00);

  /*
   * A_FULL asserts when the FIFO holds (256 - FIFO_A_FULL) samples. 0xF0 -> 240
   * trips at 16 samples = 8 measurements = one tone visit at the 7-discard +
   * 1-keep cadence, so one interrupt carries one visit (~16 Hz) with 240 samples
   * of slack behind it. The original never wrote this register, leaving it at its
   * reset value 0x7F, which asserts at 129 samples.
   */
  regWrite(0x0D, 0xF0);

  /* --- DAC/ADC OSR: DAC 128, ADC 128 -> SR_BIOZ = 256.5 sps ---
   * BG_EN, Q_EN and I_EN stay clear; biozEnable() sets them. */
  regWrite(0x20, 0xA0);

  changeReg(0x17, 0, 5, 1); /* NDIV = 0 -> 512 */
  setMdiv(512);

  cur_tone = 0;
  visit_seen = 0;
  visit_kept = 0;
}

void biozEnable(void)
{
  changeReg(0x20, 0x7, 2, 3); /* BG_EN | Q_EN | I_EN */
}

void biozDisable(void)
{
  changeReg(0x20, 0x0, 2, 3);
}

void setFreq(int freq)
{
  /*
   * KDIV only. F_BIOZ = PLL_CLK / (KDIV x DAC_OSR):
   *   KDIV = 32 -> 16809984 / (32 x 128) =   4,104 Hz
   *   KDIV =  1 -> 16809984 / ( 1 x 128) = 131,328 Hz
   * ADI's reference treats DAC_OSR, KDIV, MDIV, NDIV and ADC_OSR as one
   * coordinated sequence; this moves KDIV alone, a 32x swing. Whether both
   * endpoints sit inside the PLL's specified range is unverified.
   */
  switch (freq)
  {
  case 0:
    changeReg(0x17, 5, 4, 4); /* KDIV = 32 -> ~4 kHz */
    break;

  case 1:
    changeReg(0x17, 0, 4, 4); /* KDIV = 1 -> ~131 kHz */
    break;

  default:
    printf("# invalid tone: %d\n", freq);
    break;
  }
}

/***** Register-derived quantities *****/

uint32_t getRefClkHz(void)
{
  int reg = regRead(0x1A);
  if (reg < 0)
  {
    return 32768; /* the configured value; readback failed */
  }

  /* Both branches were identical in the original and remain so: REF_CLK_SEL
   * picks internal vs external, CLK_FREQ_SEL picks the frequency. */
  return ((reg & 0x20) == 0) ? 32000 : 32768;
}

double getSampleInterval(void)
{
  uint32_t ref_clk = getRefClkHz();

  int r17 = regRead(0x17);
  int r18 = regRead(0x18);
  int r20 = regRead(0x20);
  if (r17 < 0 || r18 < 0 || r20 < 0)
  {
    return 0.0;
  }

  int mdiv = (((r17 >> 6) & 0x03) << 8) | (r18 & 0xFF);
  double pll_clk = (double)ref_clk * (mdiv + 1);

  ndiv = (((r17 >> 5) & 0x01) == 0) ? 512.0 : 1024.0;

  const uint16_t adc_osr_table[] = {8, 16, 32, 64, 128, 256, 512, 1024};
  bioz_adc_osr = adc_osr_table[(r20 >> 3) & 0x07];

  sr_bioz = pll_clk / (ndiv * bioz_adc_osr);

  return (sr_bioz == 0.0) ? 0.0 : (1.0 / sr_bioz);
}

int getDACOSR(void)
{
  int reg = regRead(0x20);
  if (reg < 0)
  {
    return 128;
  }

  switch ((reg & 0xC0) >> 6)
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
    return 32;
  }
}

int getKDiv(void)
{
  int reg = regRead(0x17);
  if (reg < 0)
  {
    return 1;
  }

  uint8_t k_div = (uint8_t)((reg & 0x1E) >> 1);
  const uint16_t k_div_table[] = {1,   2,   4,    8,    16,   32,   64,   128,
                                  256, 512, 1024, 2048, 4096, 8192, 8192, 8192};
  return k_div_table[k_div];
}

int getMdiv(void)
{
  int r17 = regRead(0x17);
  int r18 = regRead(0x18);
  if (r17 < 0 || r18 < 0)
  {
    return 512;
  }
  return (((r17 >> 6) & 0x03) << 8) | (r18 & 0xFF);
}

double getPllClk(void)
{
  return (double)getRefClkHz() * (getMdiv() + 1);
}

double getBiozFreq(void)
{
  return getPllClk() / ((double)getKDiv() * (double)getDACOSR());
}

double getBiozGain(void)
{
  int reg = regRead(0x24);
  if (reg < 0)
  {
    return -1.0;
  }

  switch (reg & 0x03)
  {
  case 0x00:
    return 1.0;
  case 0x01:
    return 2.0;
  case 0x02:
    return 5.0;
  case 0x03:
    return 10.0;
  default:
    return -1.0;
  }
}

double getBiozCurrent_uA(void)
{
  int reg = regRead(0x22);
  if (reg < 0)
  {
    return -1.0;
  }

  uint8_t vdrv_mag_bits = (uint8_t)((reg >> 4) & 0x03);
  uint8_t idrv_rge_bits = (uint8_t)((reg >> 2) & 0x03);

  const double vdrv_mag_mv[] = {50.0, 100.0, 250.0, 500.0};
  const double r_idrv[] = {552500.0, 110500.0, 5525.0, 276250.0};

  /*
   * Returns the PEAK amplitude. The datasheet quotes drive currents in RMS, so
   * 0x28 -> 45.249 uA peak is the same current as its "32 uA" figure
   * (45.249 / sqrt(2) = 31.996). Both conventions are correct; they are just
   * different conventions.
   */
  double i_amperes = (vdrv_mag_mv[vdrv_mag_bits] / 1000.0) / r_idrv[idrv_rge_bits];
  return i_amperes * 1e6;
}

double getBiozOhmCoeff(void)
{
  const double V_REF = 1.0;
  const double TWO_OVER_PI = 2.0 / M_PI;
  const double ADC_FS = pow(2, 19);

  double gain = getBiozGain();
  double i_mag = getBiozCurrent_uA() / 1e6;

  if (gain <= 0.0 || i_mag <= 0.0)
  {
    return 0.0;
  }

  /*
   * Matches ADI's reference conversion: 2^19 x gain x (2/pi) x I_peak. Note this
   * yields the raw ratio, with no offset subtraction and no phase de-rotation --
   * the datasheet's calibration procedure supplies both, and until it is
   * implemented the absolute value carries an unknown error.
   */
  return V_REF / (ADC_FS * gain * TWO_OVER_PI * i_mag);
}

void setBiozOhmCoeff(double c)
{
  if (c > 0.0)
  {
    bioz_ohm_coeff = c;
  }
}

double getBiozOhmCoeffCached(void)
{
  return bioz_ohm_coeff;
}

void biozRefreshCache(void)
{
  double c = getBiozOhmCoeff();
  if (c > 0.0)
  {
    setBiozOhmCoeff(c);
  }
  else
  {
    /* Say so, rather than silently leaving the coefficient at 1.0 -- which would
     * report raw counts as ohms and look entirely plausible. */
    printf("# WARNING ohm coefficient readback failed, keeping %.9f\n", bioz_ohm_coeff);
  }

  getSampleInterval(); /* refreshes sr_bioz */
  bioz_freq_cached = getBiozFreq();
}

/***** Decoding *****/

bool biozToneJustSwitched(void)
{
  bool v = tone_just_switched;
  tone_just_switched = false;
  return v;
}

uint32_t biozDecodedCount(void)
{
  return decoded_count;
}

uint32_t biozKeptCount(void)
{
  return kept_count;
}

/** @brief Sign-extend a 20-bit two's-complement sample. */
static int32_t sign_extend_20(uint32_t raw)
{
  if (raw & 0x80000ul)
  {
    return (int32_t)raw - 0x100000l;
  }
  return (int32_t)raw;
}

int calcBioZ(uint8_t buf[], bool unused)
{
  (void)unused;

  uint8_t x1[3], x2[3];

  for (int i = 0; i < 3; i++)
  {
    x1[i] = buf[i];
    x2[i] = buf[i + 3];
  }

  uint8_t a = (uint8_t)(x1[0] & 0xF0);
  uint8_t b = (uint8_t)(x2[0] & 0xF0);

  uint32_t adcI, adcQ;

  /* Tag 0x1 is in-phase, 0x2 is quadrature (datasheet Table 4). */
  if ((a == 0x10) && (b == 0x20))
  {
    adcI = ((uint32_t)(x1[0] & 0x0F) << 16) | ((uint32_t)x1[1] << 8) | x1[2];
    adcQ = ((uint32_t)(x2[0] & 0x0F) << 16) | ((uint32_t)x2[1] << 8) | x2[2];
  }
  else if ((a == 0x20) && (b == 0x10))
  {
    adcI = ((uint32_t)(x2[0] & 0x0F) << 16) | ((uint32_t)x2[1] << 8) | x2[2];
    adcQ = ((uint32_t)(x1[0] & 0x0F) << 16) | ((uint32_t)x1[1] << 8) | x1[2];
  }
  else
  {
    return 3;
  }

  decoded_count++;
  visit_seen++;

  if (visit_seen <= BIOZ_SETTLE_DISCARD)
  {
    return 0; /* still settling after the tone change */
  }

  int32_t I = sign_extend_20(adcI);
  int32_t Q = sign_extend_20(adcQ);

  double coeff = getBiozOhmCoeffCached();
  double I_ohm = (double)I * coeff;
  double Q_ohm = (double)Q * coeff;

  /*
   * Timestamp derived from the kept-sample index, so it assumes nothing was ever
   * missed. Fine for reading a trace by; not a measured acquisition time, since a
   * dropped sample shifts everything after it instead of leaving a visible gap.
   */
  uint32_t t_ms = 0u;
  if (sr_bioz > 0.0)
  {
    double per_kept = (double)(BIOZ_SETTLE_DISCARD + BIOZ_KEEP_PER_VISIT) /
                      (double)BIOZ_KEEP_PER_VISIT;
    t_ms = (uint32_t)((double)sample_index * per_kept * 1000.0 / sr_bioz);
  }
  sample_index++;
  kept_count++;

  /* timestamp_ms, Q (reactive), I (resistive), tone_Hz -- same column order as
   * the full firmware's CSV, so existing plotting scripts still work. */
  printf("%lu,%.2f,%.2f,%.0f\n", (unsigned long)t_ms, Q_ohm, I_ohm, bioz_freq_cached);

  visit_kept++;
  if (visit_kept >= BIOZ_KEEP_PER_VISIT)
  {
    visit_seen = 0;
    visit_kept = 0;
    cur_tone = (uint8_t)(cur_tone ? 0u : 1u);

    setFreq((int)cur_tone);

    /* FLUSH_FIFO (0x0E bit 4, self-clearing). Without it, samples already queued
     * under the previous tone would be charged to the new tone's settling
     * budget -- which the original's interrupt counting could not prevent. */
    changeReg(0x0E, 1, 4, 1);

    bioz_freq_cached = getBiozFreq();
    tone_just_switched = true;
  }

  return 0;
}
