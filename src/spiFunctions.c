/*
 * SPI transport for the MAX30009, and FIFO framing.
 *
 * Every function here blocks. That matters: the MSDK's blocking transaction ends
 * with
 *
 *     while (!((req->spi)->intfl & MXC_F_SPI_REVA_INTFL_MST_DONE)) {}
 *
 * (spi_reva1.c, MXC_SPI_RevA1_MasterTransaction) and there is no timeout on it.
 * A second SPI transaction begun while one is in flight can therefore leave the
 * first spinning forever. Nothing in this build may touch the bus from interrupt
 * context -- see the ISR in main.c, which only sets a flag.
 */

#include "spiFunctions.h"

#include "MAX30009.h"
#include "bioZ.h"
#include "board.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_pins.h"
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
 */
static uint8_t fifo_tx[MAX30009_FIFO_READ_MAX_BYTES];
static uint8_t fifo_rx[MAX30009_FIFO_READ_MAX_BYTES];

/* An I or Q word waiting for its partner. A burst can end mid-pair, so the
 * partial pair carries into the next burst rather than being dropped. */
static uint8_t pair_word[MAX30009_FIFO_WORD_BYTES];
static uint8_t pair_tag;
static bool pair_held;

static uint32_t lost_samples;
static uint32_t tag_errors;

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
   * The original had MXC_Delay(MXC_DELAY_USEC(75)) here. It traces back to a
   * 10 ms delay in ADI's GSR demo (its spiFunctions.c), invisible at ~16 sps and
   * never a documented device requirement -- whittled down over time, removed
   * once, then silently reinstated. Left out here: if register access proves
   * unreliable without it, that is worth establishing explicitly rather than
   * carrying a delay nobody can justify.
   */
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

  /* Returns the value, or -1 on bus failure. The original returned the value in
   * a uint8_t and so could not distinguish an error code from data -- a failed
   * read looked like a plausible register value. */
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
   * the field's high bit and numBits its width, so (bit1=4, numBits=4) means
   * bits 4..1. Kept as-is so register writes stay comparable with the full
   * firmware.
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

  req.spi = SPI;
  req.txData = fifo_tx;
  req.rxData = fifo_rx;
  req.txLen = len;
  req.rxLen = len;
  req.ssIdx = 0;
  req.ssDeassert = 1;
  req.txCnt = 0;
  req.rxCnt = 0;
  req.completeCB = NULL;

  int err = MXC_SPI_MasterTransaction(&req);
  if (err != E_NO_ERROR)
  {
    printf("FIFO burst fail: %d\n", err);
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
      /* The tone changed and the FIFO was flushed. Everything left in this
       * buffer was acquired under the tone just left; decoding it would charge
       * those samples to the new tone's settling budget. */
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
