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
#include "att_api.h"
#include "dats_api.h"
#include "rtc.h"
#include "time.h"
#include <stdbool.h>

#define BIOZ_LOG_TRANSFER_MAGIC 0x4C47U
#define BIOZ_LOG_TRANSFER_VERSION 1U
#define BIOZ_LOG_TRANSFER_CHUNK 220U
#define BIOZ_LOG_TRANSFER_WINDOW 4U
#define BIOZ_LOG_TRANSFER_HEADER_LEN 8U
#define BIOZ_LOG_ATT_NOTIFY_OVERHEAD 3U

/***** Globals *****/
FATFS *fs; // FFat Filesystem Object
FATFS fs_obj;
FIL file;    // FFat File Object
FRESULT err; // FFat Result (Struct)
FILINFO fno; // FFat File Information Object
DIR dir;     // FFat Directory Object
TCHAR *FF_ERRORS[20];
BYTE work[4096];

DWORD clusters_free = 0, sectors_free = 0, sectors_total = 0, volume_sn = 0;
UINT bytes_written = 0, bytes_read = 0, mounted = 0;

static char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789,.-#'?!";
TCHAR message[MAXLEN], directory[MAXLEN], cwd[MAXLEN], filename[MAXLEN], volume_label[24],
    volume = '0';
mxc_gpio_cfg_t SDPowerEnablePin = {MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT,
                                   MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO};
char new_log_file[64];
static bool bioz_log_file_open = false;
static FIL bioz_log_read_file;
static bool bioz_log_transfer_active = false;
static bool bioz_log_transfer_read_open = false;
static bool bioz_log_transfer_read_done = false;
static uint8_t bioz_log_transfer_conn_id = 0;
static uint16_t bioz_log_transfer_seq = 0;
static uint16_t bioz_log_transfer_in_flight = 0;
static uint32_t bioz_log_transfer_sent = 0;
static uint32_t bioz_log_transfer_size = 0;
static char bioz_log_transfer_name[64];

typedef struct __attribute__((packed))
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t seq;
    uint16_t len;
    uint8_t data[BIOZ_LOG_TRANSFER_CHUNK];
} bioz_log_transfer_packet_t;

// /***** FUNCTIONS *****/

static void biozLogSendText(uint8_t connId, const char *text)
{
    if (text != NULL)
    {
        datsSendData((dmConnId_t)connId, text, (uint16_t)strlen(text));
    }
}

static uint16_t biozLogTransferPayloadLen(uint8_t connId)
{
    uint16_t mtu = AttGetMtu((dmConnId_t)connId);
    uint16_t notify_payload;

    if (mtu <= BIOZ_LOG_ATT_NOTIFY_OVERHEAD)
    {
        notify_payload = 20U;
    }
    else
    {
        notify_payload = (uint16_t)(mtu - BIOZ_LOG_ATT_NOTIFY_OVERHEAD);
    }

    if (notify_payload <= BIOZ_LOG_TRANSFER_HEADER_LEN)
    {
        return 0U;
    }

    notify_payload = (uint16_t)(notify_payload - BIOZ_LOG_TRANSFER_HEADER_LEN);
    if (notify_payload > BIOZ_LOG_TRANSFER_CHUNK)
    {
        notify_payload = BIOZ_LOG_TRANSFER_CHUNK;
    }

    return notify_payload;
}

static void biozLogTransferFinish(void)
{
    char line[128];

    bioz_log_transfer_active = false;
    bioz_log_transfer_read_done = false;
    bioz_log_transfer_in_flight = 0;
    snprintf(line, sizeof(line), "logs:read_end:%s:%lu",
             bioz_log_transfer_name, (unsigned long)bioz_log_transfer_sent);
    biozLogSendText(bioz_log_transfer_conn_id, line);
}

static bool biozLogNameIsValid(const char *name)
{
    size_t len;

    if (name == NULL)
    {
        return false;
    }

    len = strlen(name);
    if (len < 14U || len >= sizeof(bioz_log_transfer_name))
    {
        return false;
    }

    if (strncmp(name, "bioz-log-", 9U) != 0)
    {
        return false;
    }

    if (strcmp(&name[len - 4U], ".dat") != 0)
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        char c = name[i];
        bool ok = ((c >= 'a') && (c <= 'z')) ||
                  ((c >= 'A') && (c <= 'Z')) ||
                  ((c >= '0') && (c <= '9')) ||
                  (c == '-') || (c == '_') || (c == '.');
        if (!ok)
        {
            return false;
        }
    }

    return true;
}

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
    char file_extension[] = ".dat";
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
         file_extension);


    snprintf(new_log_file, MAXLEN, "%s", temp_filename); // Save to global file path
    // printf("Log file created: %s\n", new_log_file);      // <-- ADD THIS LINE

    if ((err = f_open(&file, (const TCHAR *)temp_filename, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
    {
        printf("Error creating file: %s\n", FF_ERRORS[err]);
        return err;
    }

    if ((err = f_close(&file)) != FR_OK)
    {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        return err;
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
    bioz_log_file_open = true;
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
    else
    {
        bioz_log_file_open = false;
    }

    return err;
}

int biozLogsSendList(uint8_t connId)
{
    DIR log_dir;
    FILINFO log_info;
    char line[96];

    if (!mounted)
    {
        mount();
    }

    if (err != FR_OK)
    {
        biozLogSendText(connId, "logs:error:mount");
        return err;
    }

    biozLogSendText(connId, "logs:begin");

    err = f_opendir(&log_dir, cwd);
    if (err != FR_OK)
    {
        biozLogSendText(connId, "logs:error:opendir");
        return err;
    }

    while (1)
    {
        err = f_readdir(&log_dir, &log_info);
        if (err != FR_OK || log_info.fname[0] == 0)
        {
            break;
        }

        if ((log_info.fattrib & AM_DIR) == 0 && biozLogNameIsValid(log_info.fname))
        {
            snprintf(line, sizeof(line), "logs:file:%s:%lu",
                     log_info.fname, (unsigned long)log_info.fsize);
            biozLogSendText(connId, line);
        }
    }

    f_closedir(&log_dir);
    biozLogSendText(connId, "logs:end");
    return err;
}

int biozLogTransferStart(uint8_t connId, const char *log_name)
{
    FILINFO log_info;
    char line[128];

    if (bioz_log_file_open)
    {
        biozLogSendText(connId, "logs:error:busy");
        return FR_DENIED;
    }

    if (!biozLogNameIsValid(log_name))
    {
        biozLogSendText(connId, "logs:error:name");
        return FR_INVALID_NAME;
    }

    if (bioz_log_transfer_active)
    {
        biozLogTransferCancel();
    }

    if (!mounted)
    {
        mount();
    }

    if (err != FR_OK)
    {
        biozLogSendText(connId, "logs:error:mount");
        return err;
    }

    err = f_stat(log_name, &log_info);
    if (err != FR_OK)
    {
        biozLogSendText(connId, "logs:error:not_found");
        return err;
    }

    err = f_open(&bioz_log_read_file, log_name, FA_READ);
    if (err != FR_OK)
    {
        biozLogSendText(connId, "logs:error:open");
        return err;
    }

    snprintf(bioz_log_transfer_name, sizeof(bioz_log_transfer_name), "%s", log_name);
    bioz_log_transfer_conn_id = connId;
    bioz_log_transfer_seq = 0;
    bioz_log_transfer_in_flight = 0;
    bioz_log_transfer_sent = 0;
    bioz_log_transfer_size = (uint32_t)log_info.fsize;
    bioz_log_transfer_read_open = true;
    bioz_log_transfer_read_done = false;
    bioz_log_transfer_active = true;

    snprintf(line, sizeof(line), "logs:read_begin:%s:%lu",
             bioz_log_transfer_name, (unsigned long)bioz_log_transfer_size);
    biozLogSendText(connId, line);
    return FR_OK;
}

void biozLogTransferCancel(void)
{
    if (bioz_log_transfer_active)
    {
        if (bioz_log_transfer_read_open)
        {
            f_close(&bioz_log_read_file);
            bioz_log_transfer_read_open = false;
        }
        bioz_log_transfer_active = false;
        bioz_log_transfer_read_done = false;
        bioz_log_transfer_in_flight = 0;
    }
}

void biozLogTransferAck(uint8_t connId, uint16_t next_seq)
{
    if (!bioz_log_transfer_active || connId != bioz_log_transfer_conn_id)
    {
        return;
    }

    if (next_seq <= bioz_log_transfer_seq)
    {
        bioz_log_transfer_in_flight = (uint16_t)(bioz_log_transfer_seq - next_seq);
    }
}

void biozLogTransferProcess(void)
{
    bioz_log_transfer_packet_t packet;
    UINT bytes_read = 0;
    uint16_t payload_len;

    if (!bioz_log_transfer_active)
    {
        return;
    }

    if (bioz_log_transfer_read_done)
    {
        if (bioz_log_transfer_in_flight == 0U)
        {
            biozLogTransferFinish();
        }
        return;
    }

    if (bioz_log_transfer_in_flight >= BIOZ_LOG_TRANSFER_WINDOW)
    {
        return;
    }

    payload_len = biozLogTransferPayloadLen(bioz_log_transfer_conn_id);
    if (payload_len == 0U)
    {
        biozLogSendText(bioz_log_transfer_conn_id, "logs:error:mtu");
        biozLogTransferCancel();
        return;
    }

    packet.magic = BIOZ_LOG_TRANSFER_MAGIC;
    packet.version = BIOZ_LOG_TRANSFER_VERSION;
    packet.type = 1U;

    err = f_read(&bioz_log_read_file, packet.data, payload_len, &bytes_read);
    if (err != FR_OK)
    {
        biozLogSendText(bioz_log_transfer_conn_id, "logs:error:read");
        biozLogTransferCancel();
        return;
    }

    if (bytes_read > 0U)
    {
        packet.seq = bioz_log_transfer_seq++;
        packet.len = (uint16_t)bytes_read;
        datsSendData((dmConnId_t)bioz_log_transfer_conn_id,
                     (const char *)&packet,
                     (uint16_t)(sizeof(packet) - BIOZ_LOG_TRANSFER_CHUNK + bytes_read));
        bioz_log_transfer_sent += bytes_read;
        bioz_log_transfer_in_flight++;
    }

    if (bytes_read < payload_len)
    {
        f_close(&bioz_log_read_file);
        bioz_log_transfer_read_open = false;
        bioz_log_transfer_read_done = true;
        if (bioz_log_transfer_in_flight == 0U)
        {
            biozLogTransferFinish();
        }
    }
}
