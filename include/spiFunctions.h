#ifndef SPIFUNCTIONS_H
#define SPIFUNCTIONS_H

#include <stdbool.h>
#include <stdint.h>

/* ---- MAX30009 FIFO geometry (datasheet: "FIFO Description", Table 4) ---- */

/** FIFO depth in samples. A_FULL asserts at (this - FIFO_A_FULL) samples. */
#define MAX30009_FIFO_DEPTH 256

/** Bytes per FIFO sample: 4-bit tag + 20-bit data. */
#define MAX30009_FIFO_WORD_BYTES 3

/** Register address + read command, clocked out before FIFO data comes back. */
#define MAX30009_FIFO_CMD_BYTES 2

/** FIFO samples moved per burst. Bounds the static transfer buffers. */
#define MAX30009_FIFO_READ_MAX_WORDS 64

#define MAX30009_FIFO_READ_MAX_BYTES \
  (MAX30009_FIFO_CMD_BYTES + (MAX30009_FIFO_READ_MAX_WORDS * MAX30009_FIFO_WORD_BYTES))

/*
 * Settling gap between SPI transactions. The original firmware had 75 us here and
 * worked; removing it was an unforced change and was why the first version of this
 * build produced no samples. The MAX30009 may need a minimum chip-select-high time
 * that back-to-back transactions violate -- BIAsettings() issues about thirty
 * writes in a row. Build with -DSPI_INTER_XACT_DELAY_US=0 to test that
 * hypothesis deliberately.
 */
#ifndef SPI_INTER_XACT_DELAY_US
#define SPI_INTER_XACT_DELAY_US 75
#endif

/*
 * Move FIFO bursts with DMA instead of a blocking transfer. Register access stays
 * blocking either way: it runs at configuration time and at each tone change, not
 * in the hot path.
 *
 * Build with -DBIOZ_USE_DMA=0 for the blocking path, which is the configuration
 * already known to produce correct samples. The two are meant to be
 * interchangeable, and the point of the comparison is that the printed values do
 * not change -- DMA should alter timing, not data.
 */
#ifndef BIOZ_USE_DMA
#define BIOZ_USE_DMA 1
#endif

/*
 * Multi-transaction mode keeps the driver's two DMA channels between transfers
 * rather than acquiring and releasing them every time. Off for this first DMA
 * test: nothing else here uses DMA, so all four channels are free and acquisition
 * cannot fail, and off means one fewer interaction with the blocking register path
 * that shares this SPI instance. The full firmware will want it on.
 */
#ifndef BIOZ_DMA_MT_MODE
#define BIOZ_DMA_MT_MODE 0
#endif

/*
 * Ceiling on how long a DMA burst may take before it is abandoned. A 50-byte
 * transfer at 1 MHz is about 400 us, so this is an order of magnitude of slack.
 * Bounded on purpose: waiting forever on a completion that never arrives is the
 * exact failure this build exists to avoid, and a missing DMA IRQ handler would
 * produce it.
 */
#ifndef BIOZ_DMA_TIMEOUT_US
#define BIOZ_DMA_TIMEOUT_US 10000
#endif

int initSPI(void);
int shutdownSPI(void);
uint8_t doSPI(uint8_t *tx_buf, uint8_t tx_len, uint8_t *rx_buf, uint8_t rx_len);
void regWrite(uint8_t regAddr, uint8_t val);
int regRead(uint8_t regAddr);
void changeReg(uint8_t regAddr, uint8_t val, uint8_t bit1, uint8_t numBits);

/**
 * @brief Read the FIFO fill level in samples, and optionally the overflow count.
 *
 * FIFO_DATA_COUNT is nine bits split across two registers: the MSB is
 * FIFO_COUNTER_1[7] (0x0A) and the low eight are FIFO_COUNTER_2[7:0] (0x0B).
 * The low seven bits of 0x0A are OVF_COUNTER -- samples lost since it was last
 * cleared -- a different quantity, which must not be folded into the level.
 *
 * @param ovf_out Receives OVF_COUNTER (0..127) when non-NULL.
 * @return FIFO samples available, 0..MAX30009_FIFO_DEPTH.
 */
uint16_t fifoReadLevel(uint8_t *ovf_out);

/**
 * @brief Read one burst of FIFO samples and decode complete I/Q pairs.
 *
 * @param print When true, each decoded measurement is printed.
 * @return E_NO_ERROR, or the error that aborted the burst.
 */
int fifoBurst(bool print);

/** @brief FIFO samples the device reported losing to overflow, cumulative. */
uint32_t fifoLostSamples(void);

/** @brief Words whose tag was neither in-phase nor quadrature, or unpaired. */
uint32_t fifoTagErrors(void);

/** @brief Completed DMA bursts. Zero when BIOZ_USE_DMA is 0. */
uint32_t fifoDmaTransfers(void);

/** @brief DMA bursts abandoned on timeout. Must stay 0. */
uint32_t fifoDmaTimeouts(void);

/** @brief DMA submissions or completions that reported an error. Must stay 0. */
uint32_t fifoDmaErrors(void);

#endif /* SPIFUNCTIONS_H */
