/*
 * SPI transport for the MAX30009, and FIFO framing.
 *
 * The FIFO burst can run either blocking or over DMA -- see BIOZ_USE_DMA in
 * spiFunctions.h. Register access is always blocking; it happens at
 * configuration time and at each tone change, not in the hot path, and keeping it
 * synchronous means there is only one thing to reason about.
 *
 * Nothing here may be called from interrupt context. The MSDK's blocking
 * transaction ends with
 *
 *     while (!((req->spi)->intfl & MXC_F_SPI_REVA_INTFL_MST_DONE)) {}
 *
 * (spi_reva1.c, MXC_SPI_RevA1_MasterTransaction) and there is no timeout on it, so
 * a second transaction begun while one is in flight can leave the first spinning
 * forever. Every wait this file adds is bounded, for exactly that reason.
 */

#include "spiFunctions.h"

#include "MAX30009.h"
#include "bioZ.h"
#include "board.h"
#include "dma.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "spi.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/***** Definitions *****/

#define SPI_SPEED 1000000 /* bit rate */
#define SPI MXC_SPI1

/* FIFO_DATA tag values, per datasheet Table 4 "FIFO Data Format". */
#define FIFO_TAG_IN_PHASE 0x1
#define FIFO_TAG_QUAD_PHASE 0x2

/* Whole-word sentinels, per the same table. */
#define FIFO_WORD_INVALID 0xFFFFFFul /* returned when an empty FIFO is read */
#define FIFO_WORD_MARKER 0xFFFFFEul  /* timing marker inserted via FIFO_MARK */

/*
 * Pushed when a BioZ channel is disabled while a sample is in flight. The
 * datasheet says to ignore it, and to avoid it by disabling the channel right
 * after a FIFO_DATA_RDY or A_FULL interrupt.
 */
#define FIFO_WORD_CHANNEL_OFF 0x004000ul

/***** State *****/

static mxc_spi_pins_t spi_pins;
static mxc_spi_req_t req;

/*
 * Static transfer buffers. The original declared these as variable-length arrays
 * sized from the device-reported FIFO count -- which, with that count misread,
 * could be zero (undefined behaviour) or large enough to matter on the stack.
 * A fixed buffer means the reported level can only select how much gets used.
 * Word-aligned because the DMA engine moves them.
 */
static uint8_t fifo_tx[MAX30009_FIFO_READ_MAX_BYTES] __attribute__((aligned(4)));
static uint8_t fifo_rx[MAX30009_FIFO_READ_MAX_BYTES] __attribute__((aligned(4)));

/* An I or Q word waiting for its partner. A burst can end mid-pair, so the
 * partial pair carries into the next burst rather than being dropped. */
static uint8_t pair_word[MAX30009_FIFO_WORD_BYTES];
static uint8_t pair_tag;
static bool pair_held;

static uint32_t lost_samples;
static uint32_t tag_errors;
static uint32_t dma_transfers;
static uint32_t dma_timeouts;
static uint32_t dma_errors;

#if BIOZ_USE_DMA
static volatile bool dma_done;
static volatile int dma_result;

/*
 * The SPI driver takes the lowest-numbered free DMA channels, so which ones it
 * ends up with depends on what else has claimed channels. All four are wired to
 * the shared handler rather than guessing. Without these the completion callback
 * never fires and the transfer appears to hang -- which, with an unbounded wait,
 * is indistinguishable from the freeze this whole exercise is about.
 */
void DMA0_IRQHandler(void)
{
  MXC_DMA_Handler();
}
void DMA1_IRQHandler(void)
{
  MXC_DMA_Handler();
}
void DMA2_IRQHandler(void)
{
  MXC_DMA_Handler();
}
void DMA3_IRQHandler(void)
{
  MXC_DMA_Handler();
}

/** @brief Driver completion callback. Runs in DMA interrupt context. */
static void fifoDmaComplete(void *cb_req, int result)
{
  (void)cb_req;

  dma_result = result;
  dma_done = true;
}
#endif /* BIOZ_USE_DMA */

/***** Implementation *****/

int initSPI(void)
{
  int retVal;

  spi_pins.clock = TRUE;
  spi_pins.miso = TRUE;
  spi_pins.mosi = TRUE;
  spi_pins.sdio2 = FALSE;
  spi_pins.sdio3 = FALSE;
  spi_pins.ss0 = TRUE;
  spi_pins.ss1 = FALSE;
  spi_pins.ss2 = FALSE;

  if ((retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins)) != E_NO_ERROR)
  {
    printf("SPI init error: %d\n", retVal);
    return retVal;
  }
  if ((retVal = MXC_SPI_SetMode(SPI, 0)) != E_NO_ERROR)
  {
    printf("SPI mode error: %d\n", retVal);
    return retVal;
  }
  if ((retVal = MXC_SPI_SetDataSize(SPI, 8)) != E_NO_ERROR)
  {
    printf("SPI data size error: %d\n", retVal);
    return retVal;
  }
  if ((retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD)) != E_NO_ERROR)
  {
    printf("SPI width error: %d\n", retVal);
    return retVal;
  }

#if BIOZ_USE_DMA
  /*
   * Done here, once, so channel acquisition order does not depend on which
   * peripheral happens to transfer first. The SPI driver calls MXC_DMA_Init()
   * itself on its first DMA transfer and reports an already-initialised
   * controller as E_BAD_STATE, which is not an error for us.
   */
  retVal = MXC_DMA_Init();
  if (retVal != E_NO_ERROR && retVal != E_BAD_STATE)
  {
    printf("DMA init error: %d\n", retVal);
    return retVal;
  }

  NVIC_EnableIRQ(DMA0_IRQn);
  NVIC_EnableIRQ(DMA1_IRQn);
  NVIC_EnableIRQ(DMA2_IRQn);
  NVIC_EnableIRQ(DMA3_IRQn);

  /*
   * Multi-transaction mode would let the driver keep its two DMA channels
   * between transfers instead of acquiring and releasing them each time
   * (spi_reva1.c:956-962, release at :1326-1333). Left off for this first DMA
   * test: nothing else here uses DMA, so all four channels are free and
   * acquisition cannot fail, and off means one less interaction with the blocking
   * register path that shares this SPI instance. Worth turning on once the DMA
   * path is known good -- it is the configuration the full firmware will need.
   */
  MXC_SPI_SetMTMode(SPI, BIOZ_DMA_MT_MODE);
#endif

  return E_NO_ERROR;
}

int shutdownSPI(void)
{
  int retVal = MXC_SPI_Shutdown(SPI);

  if (retVal != E_NO_ERROR)
  {
    printf("SPI shutdown error: %d\n", retVal);
  }
  return retVal;
}

uint8_t doSPI(uint8_t *tx_buf, uint8_t tx_len, uint8_t *rx_buf, uint8_t rx_len)
{
  /*
   * The original had MXC_Delay(MXC_DELAY_USEC(75)) here. It descends from a 10 ms
   * delay in ADI's GSR demo and is not a documented device requirement -- but the
   * original worked with it and the first version of this build, without it,
   * produced no samples. See SPI_INTER_XACT_DELAY_US.
   */
#if SPI_INTER_XACT_DELAY_US > 0
  MXC_Delay(MXC_DELAY_USEC(SPI_INTER_XACT_DELAY_US));
#endif

  req.spi = SPI;
  req.txData = tx_buf;
  req.rxData = rx_buf;
  req.txLen = tx_len;
  req.rxLen = rx_len;
  req.ssIdx = 0;
  req.ssDeassert = 1;
  req.txCnt = 0;
  req.rxCnt = 0;
  req.completeCB = NULL;

  int err = MXC_SPI_MasterTransaction(&req);
  if (err != E_NO_ERROR)
  {
    printf("SPI transaction fail: %d\n", err);
    return (uint8_t)err;
  }

  return E_NO_ERROR;
}

void regWrite(uint8_t regAddr, uint8_t val)
{
  uint8_t tx_buf[3] = {regAddr, 0x00, val};

  if (doSPI(tx_buf, 3, NULL, 0) != E_NO_ERROR)
  {
    printf("regWrite 0x%02X failed\n", regAddr);
  }
}

int regRead(uint8_t regAddr)
{
  uint8_t tx_buf[3] = {regAddr, 0x80, 0x00};
  uint8_t rx_buf[3] = {0};

  /* Returns the value, or -1 on bus failure. The original returned it in a
   * uint8_t and so could not distinguish an error code from data -- a failed read
   * looked like a plausible register value. */
  if (doSPI(tx_buf, 3, rx_buf, 3) != E_NO_ERROR)
  {
    return -1;
  }
  return rx_buf[2];
}

void changeReg(uint8_t regAddr, uint8_t val, uint8_t bit1, uint8_t numBits)
{
  /*
   * Preserved from the original, including its bit1/numBits convention: bit1 is
   * the field's high bit and numBits its width, so (bit1=4, numBits=4) means bits
   * 4..1. Kept as-is so register writes stay comparable with the full firmware.
   */
  int i = 0;
  uint8_t x1 = 0;
  for (i = 7; i > bit1; i--)
  {
    x1 = (uint8_t)(x1 + pow(2, i));
  }

  uint8_t x2 = 0;
  for (i = 0; i < (bit1 - numBits + 1); i++)
  {
    x2 = (uint8_t)(x2 + pow(2, i));
  }

  uint8_t newBits = (uint8_t)(x1 + x2);
  int reg1 = regRead(regAddr);
  if (reg1 < 0)
  {
    printf("changeReg 0x%02X: readback failed, write skipped\n", regAddr);
    return;
  }

  newBits = (uint8_t)(reg1 & newBits);
  val = (uint8_t)(val << (bit1 - numBits + 1));
  regWrite(regAddr, (uint8_t)(newBits + val));
}

uint16_t fifoReadLevel(uint8_t *ovf_out)
{
  int c1 = regRead(FIFO_COUNTER_1_REGISTER);
  int c2 = regRead(FIFO_COUNTER_2_REGISTER);

  if (c1 < 0 || c2 < 0)
  {
    if (ovf_out != NULL)
    {
      *ovf_out = 0;
    }
    return 0;
  }

  if (ovf_out != NULL)
  {
    *ovf_out = (uint8_t)(c1 & OVF_COUNTER_MASK);
  }

  /*
   * Bit 7 of 0x0A is the count's bit 8. Bits 6:0 are OVF_COUNTER, a different
   * quantity. The original masked with 0x7F, so it read the overflow counter as
   * the count -- and then used a sample count as a byte length, making every
   * burst about a third of the length it should have been.
   */
  uint16_t level = (uint16_t)(((uint16_t)(c1 & FIFO_DATA_COUNT_MSB_MASK)) << 1);
  level |= (uint16_t)(c2 & 0xFF);

  return level;
}

/**
 * @brief Move `len` bytes of FIFO data into fifo_rx.
 *
 * @return E_NO_ERROR on success, otherwise an MSDK error code.
 */
static int fifoTransfer(uint32_t len)
{
  req.spi = SPI;
  req.txData = fifo_tx;
  req.rxData = fifo_rx;
  req.txLen = len;
  req.rxLen = len;
  req.ssIdx = 0;
  req.ssDeassert = 1;
  req.txCnt = 0;
  req.rxCnt = 0;

#if BIOZ_USE_DMA
  req.completeCB = fifoDmaComplete;

  dma_done = false;
  dma_result = E_NO_ERROR;

  int err = MXC_SPI_MasterTransactionDMA(&req);
  if (err != E_NO_ERROR)
  {
    dma_errors++;
    return err;
  }

  /*
   * Bounded wait. The transfer is 50 bytes at 1 MHz, so ~400 us; the timeout is
   * an order of magnitude beyond that. Bounded rather than a bare spin on
   * purpose: an unbounded wait on a completion that never arrives is precisely
   * the failure this build exists to avoid, and a missing DMA IRQ handler or an
   * unacquired channel would produce exactly that.
   *
   * Waiting here rather than returning immediately keeps this a drop-in
   * replacement for the blocking transfer, so any change in the data is
   * attributable to DMA alone. Overlapping the transfer with other work is the
   * next step, not this one.
   */
  uint32_t waited_us = 0;
  while (!dma_done && waited_us < BIOZ_DMA_TIMEOUT_US)
  {
    MXC_Delay(MXC_DELAY_USEC(10));
    waited_us += 10;
  }

  if (!dma_done)
  {
    dma_timeouts++;
    /* Abort so the driver releases its channels and the next attempt starts
     * clean, rather than inheriting a transfer that never finished. */
    MXC_SPI_AbortAsync(SPI);
    printf("# DMA timeout after %lu us (len=%lu). Transfer abandoned.\n",
           (unsigned long)waited_us, (unsigned long)len);
    return E_TIME_OUT;
  }

  if (dma_result != E_NO_ERROR)
  {
    dma_errors++;
    return dma_result;
  }

  dma_transfers++;
  return E_NO_ERROR;
#else
  req.completeCB = NULL;
  return MXC_SPI_MasterTransaction(&req);
#endif
}

int fifoBurst(bool print)
{
  uint8_t ovf = 0;
  uint16_t level = fifoReadLevel(&ovf);

  if (ovf != 0)
  {
    /* FIFO_RO defaults to 1, so the device silently overwrote the oldest
     * samples. Alignment across the hole is not guaranteed. */
    lost_samples += ovf;
    pair_held = false;
  }

  if (level == 0)
  {
    return E_NO_ERROR;
  }

  uint16_t words = level;
  if (words > MAX30009_FIFO_READ_MAX_WORDS)
  {
    words = MAX30009_FIFO_READ_MAX_WORDS;
  }

  uint32_t len = MAX30009_FIFO_CMD_BYTES + ((uint32_t)words * MAX30009_FIFO_WORD_BYTES);

  memset(fifo_tx, 0, len);
  fifo_tx[0] = FIFO_DATA_REGISTER_REGISTER;
  fifo_tx[1] = 0x80; /* read */
  memset(fifo_rx, 0, len);

  int err = fifoTransfer(len);
  if (err != E_NO_ERROR)
  {
    printf("# FIFO burst fail: %d\n", err);
    return err;
  }

  /*
   * Fixed three-byte strides from the first data byte, keyed on each word's tag.
   * The original scanned for non-zero bytes to find frame boundaries, which only
   * appeared to work because the read length was wrong and the buffer held
   * trailing padding -- sample data legitimately contains zero bytes.
   */
  for (uint16_t w = 0; w < words; w++)
  {
    const uint8_t *word = &fifo_rx[MAX30009_FIFO_CMD_BYTES + (w * MAX30009_FIFO_WORD_BYTES)];
    uint32_t raw = ((uint32_t)word[0] << 16) | ((uint32_t)word[1] << 8) | (uint32_t)word[2];

    if (raw == FIFO_WORD_INVALID)
    {
      break; /* read past valid data; nothing after this is meaningful */
    }
    if (raw == FIFO_WORD_MARKER || raw == FIFO_WORD_CHANNEL_OFF)
    {
      continue;
    }

    uint8_t tag = (uint8_t)(word[0] >> 4);
    if (tag != FIFO_TAG_IN_PHASE && tag != FIFO_TAG_QUAD_PHASE)
    {
      tag_errors++;
      continue;
    }

    if (!pair_held)
    {
      memcpy(pair_word, word, MAX30009_FIFO_WORD_BYTES);
      pair_tag = tag;
      pair_held = true;
      continue;
    }

    if (pair_tag == tag)
    {
      /* Two same-phase words running: the partner was lost. Keep the newer word
       * so the stream re-pairs rather than staying skewed by one. */
      tag_errors++;
      memcpy(pair_word, word, MAX30009_FIFO_WORD_BYTES);
      pair_tag = tag;
      continue;
    }

    uint8_t pair[2 * MAX30009_FIFO_WORD_BYTES];
    memcpy(&pair[0], pair_word, MAX30009_FIFO_WORD_BYTES);
    memcpy(&pair[MAX30009_FIFO_WORD_BYTES], word, MAX30009_FIFO_WORD_BYTES);
    pair_held = false;

    calcBioZ(pair, print);

    if (biozToneJustSwitched())
    {
      /* The tone changed and the FIFO was flushed. Anything left in this buffer
       * was acquired under the tone just left; decoding it would charge those
       * samples to the new tone's settling budget. */
      break;
    }
  }

  return E_NO_ERROR;
}

uint32_t fifoLostSamples(void)
{
  return lost_samples;
}

uint32_t fifoTagErrors(void)
{
  return tag_errors;
}

uint32_t fifoDmaTransfers(void)
{
  return dma_transfers;
}

uint32_t fifoDmaTimeouts(void)
{
  return dma_timeouts;
}

uint32_t fifoDmaErrors(void)
{
  return dma_errors;
}
