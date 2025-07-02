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
#include "cli.h"

#include "spiFunctions.h"

#include "sdhc.h"
#include "user-cli.h"
#include <stdlib.h>
#import "time.h"
#import "rtc.h"
const command_t user_commands[] = {
    {"size", "size", "Find the Size of the SD Card and Free Space",
     handle_size},
    {"format", "format", "Format the Card", handle_format},
    {"mount", "mount", "Manually Mount Card", handle_mount},
    {"ls", "ls", "list the contents of the current directory", handle_ls},
    {"mkdir", "mkdir <directory name>", "Create a directory", handle_mkdir},
    {"file_create", "file_create <file name> <number of bytes to add>",
     "Create a file of random data", handle_createfile},
    {"cd", "cd <directory name>", "Move into a directory", handle_cd},
    {"add_data", "add_data <file name> <number of bytes to add>",
     "Add random Data to an Existing File", handle_add_data},
    {"del", "del <file name>", "Delete a file", handle_del},
    {"unmount", "unmount", "Unmount card", handle_unmount},
    {"start", "start", "Start recording", handle_start},
    {"stop", "stop", "Stop recording", handle_stop},
};

extern int sample_index; // Declare sample_index as extern to access it in other files
const unsigned int num_user_commands =
    sizeof(user_commands) / sizeof(command_t);

int handle_size(int argc, char *argv[])
{
  if (argc != 1)
  {
    printf("Incorrect usage. Too many parameters.\n");
    return E_INVALID;
  }

  return getSize();
}

int handle_format(int argc, char *argv[])
{
  if (argc != 1)
  {
    printf("Incorrect usage. Too many parameters.\n");
    return E_INVALID;
  }

  return formatSDHC();
}

int handle_mount(int argc, char *argv[])
{
  if (argc != 1)
  {
    printf("Incorrect usage. Too many parameters.\n");
    return E_INVALID;
  }

  return mount();
}

int handle_ls(int argc, char *argv[])
{
  if (argc != 1)
  {
    printf("Incorrect usage. Too many parameters.\n");
    return E_INVALID;
  }

  return ls();
}

int handle_mkdir(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("Incorrect usage. Please provide directory name.\n");
    return E_INVALID;
  }

  return mkdir(argv[1]);
}

int handle_createfile(int argc, char *argv[])
{
  if (argc != 3)
  {
    printf("Incorrect usage. Please provide filename and length.\n");
    return E_INVALID;
  }

  unsigned int length = atoi(argv[2]);
  return createFile(argv[1], length);
}

int handle_cd(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("Incorrect usage. Please provide directory name.\n");
    return E_INVALID;
  }

  return cd(argv[1]);
}

int handle_add_data(int argc, char *argv[])
{
  if (argc != 3)
  {
    printf("Incorrect usage. Please provide filename and length.\n");
    return E_INVALID;
  }

  unsigned int length = atoi(argv[2]);
  return appendFile(argv[1], length);
}

int handle_del(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("Incorrect usage. Please provide filename.\n");
    return E_INVALID;
  }

  return deleteFile(argv[1]);
}

int handle_unmount(int argc, char *argv[])
{
  if (argc != 1)
  {
    printf("Incorrect usage. Too many parameters.\n");
    return E_INVALID;
  }

  return umount();
}
int handle_start(int argc, char *argv[])
{
  int year = 1970, month = 1, day = 1;
  int hour = 0, min = 0, sec = 0;

  if (argc == 3)
  {
    char datetime_str[32];
    snprintf(datetime_str, sizeof(datetime_str), "%s %s", argv[1], argv[2]);

    if (sscanf(datetime_str, "%d-%d-%d %d-%d-%d", &year, &month, &day, &hour, &min, &sec) != 6)
    {
      printf("Invalid datetime format.\n");
      return E_INVALID;
    }
  }
  else if (argc == 1)
  {
    // printf("Default start: setting RTC to 1970-01-01 00:00:00\n");
  }
  else
  {
    printf("Usage: start [YYYY-MM-DD HH-MM-SS]\n");
    return E_INVALID;
  }

  struct tm timeinfo = {
      .tm_year = year - 1900,
      .tm_mon = month - 1,
      .tm_mday = day,
      .tm_hour = hour,
      .tm_min = min,
      .tm_sec = sec};

  time_t rawtime = mktime(&timeinfo);
  if (rawtime == -1)
  {
    printf("Failed to convert time.\n");
    return E_INVALID;
  }

  MXC_RTC_Stop();

  if (MXC_RTC_Init((uint32_t)rawtime, 0) != E_NO_ERROR)
  {
    printf("RTC init failed!\n");
    return E_UNKNOWN;
  }

  MXC_Delay(MSEC(10));

  if (MXC_RTC_Start() != E_NO_ERROR)
  {
    printf("RTC start failed!\n");
    return E_UNKNOWN;
  }

  MXC_Delay(MSEC(10));

  uint32_t sec_read;
  if (MXC_RTC_GetSeconds(&sec_read) != E_NO_ERROR)
  {
    printf("RTC read failed!\n");
    return E_UNKNOWN;
  }

  // printf("RTC started at UNIX time: %lu\n", sec_read);

  sample_index = 0;

  FRESULT err;
  if ((err = createNextBiozLogFile()) != FR_OK)
    return err;
  if ((err = openLogFile()) != FR_OK)
    return err;

  changeReg(0x20, 0x7, 2, 3);
  return E_NO_ERROR;
}

int handle_stop(int argc, char *argv[])
{
  if (argc != 1)
  {
    printf("Incorrect usage. No parameters needed.\n");
    return E_INVALID;
  }

  // Disable sampling
  changeReg(0x20, 0x0, 2, 3);

  // Close file
  closeLogFile();

  return E_NO_ERROR;
}
