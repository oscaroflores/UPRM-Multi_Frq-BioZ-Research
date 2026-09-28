# Minimal BioZ acquisition spike.
#
# Deliberately has no BLE stack, no SD card, and no CLI: just the MAX32655 talking
# to the MAX30009 and printing samples to the console. The point is a reproducer
# small enough to trust. If this runs indefinitely without freezing, the fault in
# the full firmware lies in one of the subsystems absent here, and they can be
# added back one at a time.
#
# Deliberately NOT enabled (each was on in the full firmware):
#   LIB_SDHC / FATFS_VERSION  -- FatFS, called from interrupt context there
#   LIB_CLI                   -- the CLI handler IS the UART interrupt vector
#   LIB_CORDIO + INIT_*       -- BLE stack and its WSF scheduler
#
# Any board with the MAX30009 wired to SPI1 works, since nothing here needs the
# SD slot; the full project's FTHR_Apps_P1 requirement came from LIB_SDHC.

# Console output only; no trace library.
TRACE = 0

# ---------------------------------------------------------------------------
# Build-time toggles
# ---------------------------------------------------------------------------
#
# Move FIFO bursts with DMA (1) or a blocking transfer (0). The blocking path is
# the one already known to produce correct samples, so it is the reference: the
# two are meant to be interchangeable, and the test is that the printed values do
# not change. DMA should alter timing, not data.
#
#   make BIOZ_USE_DMA=0      blocking reference
#   make                     DMA (default)
#
BIOZ_USE_DMA ?= 1
PROJ_CFLAGS += -DBIOZ_USE_DMA=$(BIOZ_USE_DMA)

# Keep the driver's two DMA channels between transfers instead of acquiring and
# releasing them each time. Off for the first DMA test -- nothing else here uses
# DMA, so acquisition cannot fail, and off means one fewer interaction with the
# blocking register path sharing this SPI instance.
BIOZ_DMA_MT_MODE ?= 0
PROJ_CFLAGS += -DBIOZ_DMA_MT_MODE=$(BIOZ_DMA_MT_MODE)

# Settling gap between SPI transactions, microseconds. The original firmware had
# 75 us and worked; dropping it was why the first version of this build produced
# no samples. Set to 0 to test that hypothesis deliberately.
SPI_INTER_XACT_DELAY_US ?= 75
PROJ_CFLAGS += -DSPI_INTER_XACT_DELAY_US=$(SPI_INTER_XACT_DELAY_US)
