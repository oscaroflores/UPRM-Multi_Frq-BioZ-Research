#ifndef BIOZ_H
#define BIOZ_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Measurements discarded after each tone change before one is kept.
 *
 * Empirically the minimum at which I/Q come out valid for this register
 * configuration. The cost is paid per tone *switch*, not per sample, so keeping
 * more than one measurement per visit amortises it: the effective total rate is
 * 256.5 * N / (7 + N) sps. Left at 1 kept per visit here to match the full
 * firmware's cadence.
 */
#define BIOZ_SETTLE_DISCARD 7
#define BIOZ_KEEP_PER_VISIT 1

/** Sample rate in samples/second, derived from the PLL and OSR registers. */
extern double sr_bioz;

/** Configure the AFE for multi-frequency BIA / respiration. */
void BIAsettings(void);

/** Enable the bandgap, I and Q channels. Starts measurement. */
void biozEnable(void);

/** Clear them. Stops measurement. */
void biozDisable(void);

/**
 * @brief Select the excitation tone.
 * @param freq 0 for the low tone (~4.1 kHz), 1 for the high tone (~131 kHz).
 */
void setFreq(int freq);

/** Set the M divider, which spans two registers. */
void setMdiv(int val);

/** Re-read the registers behind the ohm coefficient and the tone frequency. */
void biozRefreshCache(void);

/**
 * @brief Decode one I/Q pair, apply the settling policy, print if kept.
 *
 * Switches tone and flushes the AFE FIFO once a full visit has been kept, so
 * samples taken under the previous tone can never be charged to the next one's
 * settling budget.
 *
 * @param buf Six bytes: two FIFO words, either order; the tags disambiguate.
 * @return 0 if decoded, 3 if the tags were not a valid I/Q pair.
 */
int calcBioZ(uint8_t buf[], bool unused);

/**
 * @brief Read and clear the "tone just changed" flag.
 *
 * The FIFO reader uses this to stop decoding the rest of a burst: anything after
 * the switch belongs to the tone just left, and the FIFO has been flushed.
 */
bool biozToneJustSwitched(void);

/** @brief Measurements decoded since boot, and how many were kept. */
uint32_t biozDecodedCount(void);
uint32_t biozKeptCount(void);

/* Register-derived quantities. Each performs SPI reads, so none may be called
 * from interrupt context. */
uint32_t getRefClkHz(void);
double getSampleInterval(void);
int getDACOSR(void);
int getKDiv(void);
int getMdiv(void);
double getPllClk(void);
double getBiozFreq(void);
double getBiozGain(void);
double getBiozCurrent_uA(void);
double getBiozOhmCoeff(void);

void setBiozOhmCoeff(double c);
double getBiozOhmCoeffCached(void);

#endif /* BIOZ_H */
