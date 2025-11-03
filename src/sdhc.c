/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "sdhc.h"
#include "rtc.h"
#include "time.h"
/***** Globals *****/
FATFS *fs; // FFat Filesystem Object
FATFS fs_obj;
FIL file;    // FFat File Object
FRESULT err; // FFat Result (Struct)
FILINFO fno; // FFat File Information Object
DIR dir;     // FFat Directory Object
TCHAR *FF_ERRORS[20];
BYTE work[4096];
FIL imuFile;             // File handle for IMU log
UINT imu_bytes_written;
DWORD clusters_free = 0, sectors_free = 0, sectors_total = 0, volume_sn = 0;
UINT bytes_written = 0, bytes_read = 0, mounted = 0;

static char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789,.-#'?!";
TCHAR message[MAXLEN], directory[MAXLEN], cwd[MAXLEN], filename[MAXLEN], volume_label[24],
    volume = '0';
mxc_gpio_cfg_t SDPowerEnablePin = {MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT,
                                   MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO};
char new_log_file[64];
char imu_log_file[64];

// /***** FUNCTIONS *****/

void generateMessage(unsigned length)
{
    for (int i = 0; i < length; i++)
    {
        /*Generate some random data to put in file*/
        message[i] = charset[rand() % (sizeof(charset) - 1)];
    }
}

void setMessage(const char *m)
{
    // Copy the user-defined message into the global message buffer
    snprintf(message, MAXLEN, "%s", m);
}

int mount()
{
    fs = &fs_obj;

    if ((err = f_mount(fs, "", 1)) != FR_OK)
    { // Mount the default drive to fs now
        printf("Error opening SD card: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    }
    else
    {
        printf("SD card mounted.\n");
        mounted = 1;
    }

    f_getcwd(cwd, sizeof(cwd)); // Set the Current working directory

    return err;
}

int umount()
{
    if ((err = f_mount(NULL, "", 0)) != FR_OK)
    { // Unmount the default drive from its mount point
        printf("Error unmounting volume: %s\n", FF_ERRORS[err]);
    }
    else
    {
        printf("SD card unmounted.\n");
        mounted = 0;
    }

    return err;
}

int formatSDHC()
{
    printf("\n\n*****THE DRIVE WILL BE FORMATTED IN 5 SECONDS*****\n");
    printf("**************PRESS ANY KEY TO ABORT**************\n\n");
    MXC_UART_ClearRXFIFO(MXC_UART0);
    MXC_Delay(MSEC(5000));

    if (MXC_UART_GetRXFIFOAvailable(MXC_UART0) > 0)
    {
        return E_ABORT;
    }

    printf("FORMATTING DRIVE\n");

    MKFS_PARM format_options = {.fmt = FM_ANY};

    if ((err = f_mkfs("", &format_options, work, sizeof(work))) != FR_OK)
    {
        printf("Error formatting SD card: %s\n", FF_ERRORS[err]);
    }
    else
    {
        printf("Drive formatted.\n");
    }

    mount();

    if ((err = f_setlabel("MAXIM")) != FR_OK)
    {
        printf("Error setting drive label: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    }

    umount();

    return err;
}

int getSize()
{
    if (!mounted)
    {
        mount();
    }

    if ((err = f_getfree(&volume, &clusters_free, &fs)) != FR_OK)
    {
        printf("Error finding free size of card: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    }

    sectors_total = (fs->n_fatent - 2) * fs->csize;
    sectors_free = clusters_free * fs->csize;

    printf("Disk Size: %u bytes\n", sectors_total / 2);
    printf("Available: %u bytes\n", sectors_free / 2);

    return err;
}

int ls()
{
    if (!mounted)
    {
        mount();
    }

    printf("Listing Contents of %s - \n", cwd);

    if ((err = f_opendir(&dir, cwd)) == FR_OK)
    {
        while (1)
        {
            err = f_readdir(&dir, &fno);

            if (err != FR_OK || fno.fname[0] == 0)
            {
                break;
            }

            printf("%s/%s", cwd, fno.fname);

            if (fno.fattrib & AM_DIR)
            {
                printf("/");
            }

            printf("\n");
        }

        f_closedir(&dir);
    }
    else
    {
        printf("Error opening directory!\n");
        return err;
    }

    printf("\nFinished listing contents\n");

    return err;
}

int createFile(char *file_name, unsigned int length)
{
    // unsigned int length = 128;

    if (!mounted)
    {
        mount();
    }

    snprintf(filename, MAXLEN, "%s", file_name);

    if (length > MAXLEN)
    {
        printf("Error. File size limit for this example is %d bytes.\n", MAXLEN);
        return FR_INVALID_PARAMETER;
    }

    printf("Creating file %s with length %d\n", filename, length);

    if ((err = f_open(&file, (const TCHAR *)filename, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
    {
        printf("Error opening file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    // printf("File opened!\n");

    generateMessage(length);

    if ((err = f_write(&file, &message, length, &bytes_written)) != FR_OK)
    {
        printf("Error writing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("%d bytes written to file!\n", bytes_written);

    if ((err = f_close(&file)) != FR_OK)
    {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("File Closed!\n");
    return err;
}

int appendFile(char *file_name, unsigned int length)
{
    if (!mounted)
    {
        mount();
    }

    snprintf(filename, MAXLEN, "%s", file_name);

    if ((err = f_stat((const TCHAR *)filename, &fno)) == FR_NO_FILE)
    {
        printf("File %s doesn't exist!\n", (const TCHAR *)filename);
        return err;
    }

    if (length > MAXLEN)
    {
        printf("Error. Size limit for this example is %d bytes.\n", MAXLEN);
        return FR_INVALID_PARAMETER;
    }

    if ((err = f_open(&file, (const TCHAR *)filename, FA_OPEN_APPEND | FA_WRITE)) != FR_OK)
    {
        printf("Error opening file %s\n", FF_ERRORS[err]);
        return err;
    }

    // printf("File opened!\n");

    // generateMessage(length);

    if ((err = f_write(&file, &message, length, &bytes_written)) != FR_OK)
    {
        printf("Error writing file: %s\n", FF_ERRORS[err]);
        return err;
    }

    // printf("%d bytes written to file\n", bytes_written);

    if ((err = f_close(&file)) != FR_OK)
    {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        return err;
    }

    // printf("File closed.\n");
    return err;
}

int mkdir(char *dir_name)
{
    if (!mounted)
    {
        mount();
    }

    snprintf(directory, MAXLEN, "%s", dir_name);

    err = f_stat((const TCHAR *)directory, &fno);

    if (err == FR_NO_FILE)
    {
        printf("Creating directory...\n");

        if ((err = f_mkdir((const TCHAR *)directory)) != FR_OK)
        {
            printf("Error creating directory: %s\n", FF_ERRORS[err]);
            f_mount(NULL, "", 0);
            return err;
        }
        else
        {
            printf("Directory %s created.\n", directory);
        }
    }
    else
    {
        printf("Directory already exists.\n");
    }

    return err;
}

int cd(char *dir_name)
{
    if (!mounted)
    {
        mount();
    }

    snprintf(directory, MAXLEN, "%s", dir_name);

    if ((err = f_stat((const TCHAR *)directory, &fno)) == FR_NO_FILE)
    {
        printf("Directory doesn't exist (Did you mean mkdir?)\n");
        return err;
    }

    if ((err = f_chdir((const TCHAR *)directory)) != FR_OK)
    {
        printf("Error in chdir: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("Changed to %s\n", directory);
    f_getcwd(cwd, sizeof(cwd));

    return err;
}

int deleteFile(char *file_name)
{
    if (!mounted)
    {
        mount();
    }

    snprintf(filename, MAXLEN, "%s", file_name);

    if ((err = f_stat((const TCHAR *)filename, &fno)) == FR_NO_FILE)
    {
        printf("File or directory doesn't exist\n");
        return err;
    }

    if ((err = f_unlink(filename)) != FR_OK)
    {
        printf("Error deleting file\n");
        return err;
    }

    printf("Deleted file %s\n", filename);
    return err;
}

int createNextBiozLogFile()
{
    if (!mounted)
    {
        mount();
    }

    int max_n = -1;
    char file_prefix[] = "bioz-log-";
    char file_extension[] = ".csv";
    char temp_filename[MAXLEN];

    if ((err = f_opendir(&dir, cwd)) == FR_OK)
    {
        while (1)
        {
            err = f_readdir(&dir, &fno);
            if (err != FR_OK || fno.fname[0] == 0)
                break;

            if (strncmp(fno.fname, file_prefix, strlen(file_prefix)) == 0)
            {
                char *number_part = fno.fname + strlen(file_prefix);
                char *dash = strchr(number_part, '-');
                if (dash && strchr(dash, '.'))
                {
                    int current_n = atoi(dash + 1);
                    if (current_n > max_n)
                        max_n = current_n;
                }
            }
        }
        f_closedir(&dir);
    }
    else
    {
        printf("Error opening directory: %s\n", FF_ERRORS[err]);
        return err;
    }

    int next_n = max_n + 1;

    // Get RTC raw second count (since boot or set)
    uint32_t sec;
    if (MXC_RTC_GetSeconds(&sec) != E_NO_ERROR)
    {
        printf("RTC read failed.\n");
        return -1;
    }
    time_t rawtime = sec;
    struct tm *timeinfo = localtime(&rawtime);

    snprintf(temp_filename, MAXLEN, "%s%04d%02d%02d-%02d%02d%02d%s",
             file_prefix,
             timeinfo->tm_year + 1900,
             timeinfo->tm_mon + 1,
             timeinfo->tm_mday,
             timeinfo->tm_hour,
             timeinfo->tm_min,
             timeinfo->tm_sec,
             file_extension);snprintf(temp_filename, MAXLEN, "%s%04d%02d%02d-%02d%02d%02d%s",
         file_prefix,
         timeinfo->tm_year + 1900,
         timeinfo->tm_mon + 1,
         timeinfo->tm_mday,
         timeinfo->tm_hour,
         timeinfo->tm_min,
         timeinfo->tm_sec,
         file_extension);


    snprintf(new_log_file, MAXLEN, "%s", temp_filename); // Save to global file path
    // printf("Log file created: %s\n", new_log_file);      // <-- ADD THIS LINE

    if ((err = f_open(&file, (const TCHAR *)temp_filename, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
    {
        printf("Error creating file: %s\n", FF_ERRORS[err]);
        return err;
    }

    const char *csv_header = "timestamp,Q,I,F\n";
    if ((err = f_write(&file, csv_header, strlen(csv_header), &bytes_written)) != FR_OK)
    {
        printf("Error writing CSV header: %s\n", FF_ERRORS[err]);
        f_close(&file);
        return err;
    }

    if ((err = f_close(&file)) != FR_OK)
    {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        return err;
    }

    return FR_OK;
}
int createNextIMULogFile()
{
    if (!mounted) {
        mount();
    }

    int err;
    FRESULT res;
    char file_prefix[] = "imu-log-";
    char file_extension[] = ".csv";
    char temp_filename[MAXLEN];

    // Get RTC seconds
    uint32_t sec;
    if (MXC_RTC_GetSeconds(&sec) != E_NO_ERROR) {
        printf("RTC read failed.\n");
        return -1;
    }
    time_t rawtime = sec;
    struct tm *timeinfo = localtime(&rawtime);

    snprintf(temp_filename, MAXLEN, "%s%04d%02d%02d-%02d%02d%02d%s",
             file_prefix,
             timeinfo->tm_year + 1900,
             timeinfo->tm_mon + 1,
             timeinfo->tm_mday,
             timeinfo->tm_hour,
             timeinfo->tm_min,
             timeinfo->tm_sec,
             file_extension);

    snprintf(imu_log_file, MAXLEN, "%s", temp_filename);

    if ((res = f_open(&imuFile, (const TCHAR *)temp_filename, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK) {
        printf("Error creating IMU file: %s\n", FF_ERRORS[res]);
        return res;
    }

    const char *csv_header = "ax,ay,az\n";
    if ((res = f_write(&imuFile, csv_header, strlen(csv_header), &imu_bytes_written)) != FR_OK) {
        printf("Error writing IMU header: %s\n", FF_ERRORS[res]);
        f_close(&imuFile);
        return res;
    }

    if ((res = f_close(&imuFile)) != FR_OK) {
        printf("Error closing IMU file: %s\n", FF_ERRORS[res]);
        return res;
    }

    return FR_OK;
}

void waitCardInserted()
{
    // On the MAX78000FTHR board, P0.12 will be pulled low when a card is inserted.
    mxc_gpio_cfg_t cardDetect;
    cardDetect.port = MXC_GPIO0;
    cardDetect.mask = MXC_GPIO_PIN_12;
    cardDetect.func = MXC_GPIO_FUNC_IN;
    cardDetect.pad = MXC_GPIO_PAD_NONE;
    cardDetect.vssel = MXC_GPIO_VSSEL_VDDIOH;

    MXC_GPIO_Config(&cardDetect);

    // Exit function if card is already inserted
    if (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_12) == 0)
    {
        return;
    }

    while (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_12) != 0)
    {
        // Spin waiting for card to be inserted.
    }

    // Card has been detected, exit the function.
}
int openLogFile()
{
    if (!mounted)
        mount();

    err = f_open(&file, new_log_file, FA_OPEN_APPEND | FA_WRITE);

    if (err != FR_OK)
    {
        printf("Error opening log file: %s\n", FF_ERRORS[err]);
        return err;
    }
    return FR_OK;
}
int openIMULogFile(const char *imu_filename)
{
    if (!mounted)
        mount();

    FRESULT err = f_open(&imuFile, imu_filename, FA_OPEN_APPEND | FA_WRITE);
    if (err != FR_OK) {
        printf("Error opening IMU log: %s\n", FF_ERRORS[err]);
        return err;
    }

    printf("Opened IMU log: %s\n", imu_filename);
    return FR_OK;
}

int closeLogFile()
{
    err = f_sync(&file);
    if (err != FR_OK)
    {
        printf("Error syncing log file: %s\n", FF_ERRORS[err]);
    }

    err = f_close(&file);
    if (err != FR_OK)
    {
        printf("Error closing log file: %s\n", FF_ERRORS[err]);
    }
    err = f_sync(&imuFile);
    return err;
}
