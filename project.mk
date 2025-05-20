# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
# Add your config here!
ifneq ($(BOARD),FTHR_Apps_P1)
$(error ERR_NOTSUPPORTED: This project requires an SD card slot and is only supported for the MAX32655FTHR)
endif

# Enable SDHC library
LIB_SDHC = 1
# Use FatFS version R0.15
FATFS_VERSION = ff15

# Enable CLI library
LIB_CLI = 1