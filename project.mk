# Minimal BioZ acquisition spike.
#
# Deliberately has no BLE stack, no SD card, and no CLI -- just the MAX32655
# talking to the MAX30009 and printing samples to the console. The point is a
# reproducer small enough to trust: if this runs indefinitely without freezing,
# the fault lives in one of the subsystems that is absent here, and they can be
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

# Keep the console UART's default 115200 baud (board.h CONSOLE_BAUD).
