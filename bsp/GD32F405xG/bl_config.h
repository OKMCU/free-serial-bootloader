/*******************************************************************************
 * Copyright (c) 2021-2022, OKMCU Development Team 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Website: http://www.okmcu.com
 *
 * File Description: Template description.
 *
 * Change Logs:
 * Date         Author       Notes    
 * 2021-10-12   Wentao SUN   first version
 * 
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BL_CONFIG_H
#define __BL_CONFIG_H
/* Compile options -----------------------------------------------------------*/
#define BL_USE_STDLIB                 1
/* Includes ------------------------------------------------------------------*/
#if (BL_USE_STDLIB > 0)
#include <stdint.h>
#include <string.h>
#endif
/* Exported constants --------------------------------------------------------*/
#define MCU_PART_NUMBER                     "GD32F405xG"
#define COMMIT_IMG_SIGNATUR                 0xAABBCCDD
#define BLDR_PROG_BUILD_TIME                "2021-10-12 19:15:58"
#define BLDR_PROG_MAJOR_VER                 0
#define BLDR_PROG_MINOR_VER                 0
#define BLDR_PROG_BUILD_VER                 1
#define BLDR_PROG_FLASH_ADDR                0x08000000
#define BLDR_PROG_SIZE                      (16ul*1024ul)
#define FLASH_START_ADDR                    0x08000000
#define FLASH_SIZE                          (1024ul*1024ul)
#define FLASH_MASS_ERASE_TIME               32000
#define FLASH_SECTOR_NUM                    12
#define FLASH_SECTOR_ERASE_TIME             8000
#define FLASH_SECTOR_PROG_TIME              6000
#define FLASH_READ_QWORD                    1
#define FLASH_READ_DWORD                    1
#define FLASH_READ_WORD                     1
#define FLASH_READ_BYTE                     1
#define FLASH_PROG_QWORD                    1
#define FLASH_PROG_DWORD                    1
#define FLASH_PROG_WORD                     1
#define FLASH_PROG_BYTE                     1
#define FLASH_ADDR_APPCODE_START            0x08008000
#define EEPROM_START_ADDR                   0x08004000
#define EEPROM_SIZE                         0x00004000
#define EEPROM_MASS_ERASE_TIME              800
#define EEPROM_SECTOR_NUM                   1
#define EEPROM_SECTOR_ERASE_TIME            800
#define EEPROM_SECTOR_PROG_TIME             600
#define EEPROM_READ_QWORD                   1
#define EEPROM_READ_DWORD                   1
#define EEPROM_READ_WORD                    1
#define EEPROM_READ_BYTE                    1
#define EEPROM_PROG_QWORD                   1
#define EEPROM_PROG_DWORD                   1
#define EEPROM_PROG_WORD                    1
#define EEPROM_PROG_BYTE                    1
#define EEPROM_ADDR_COMMIT_IMG_SIGNATURE   (EEPROM_SIZE-4)

#define HAL_UART_BAUDRATE                   115200
#define HAL_WDG_TIMEOUT                     1000


/* Exported types ------------------------------------------------------------*/
#if (BL_USE_STDLIB == 0)
typedef unsigned char                       uint8_t;
typedef unsigned int                        uint16_t;
typedef unsigned long int                   uint32_t;
typedef char                                int8_t;
typedef int                                 int16_t;
typedef long int                            int32_t;
#endif
/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* __BL_CONFIG_H */

/******************************** END OF FILE *********************************/

