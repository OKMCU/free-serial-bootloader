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
#define HAL_UART_BAUDRATE                   115200ul
#define HAL_WDG_ENABLE                      0
#define HAL_WDG_TIMEOUT                     2000
#define CRC8_MAXIM_ALGO_SEL                 1
#define DEVICE_ADDR                         0xAA

#define MCU_PART_NUMBER                     "GD32E232x8"
#define COMMIT_IMG_SIGNATURE                 0xAABBCCDD
#define BLDR_PROG_BUILD_TIME                "2021-10-25 09:38:58"
#define BLDR_PROG_MAJOR_VER                 0
#define BLDR_PROG_MINOR_VER                 0
#define BLDR_PROG_BUILD_VER                 1
#define BLDR_PASSWORD                       "default_password"

#define FLASH_ADDR_START                    0x08000000
#define FLASH_ADDR_END                      0x0800FFFF

#define FLASH_ADDR_BLDR_START               FLASH_ADDR_START
#define FLASH_ADDR_BLDR_END                 0x08001BFF
#define FLASH_ADDR_VIRTUAL_EEPROM_START     0x08001C00
#define FLASH_ADDR_VIRTUAL_EEPROM_END       0x08001FFF
#define FLASH_ADDR_APPCODE_START            0x08002000
#define FLASH_ADDR_APPCODE_END              FLASH_ADDR_END

#define FLASH_PAGE_NUM                      64
#define FLASH_PAGE_ERASE_TIME               4
#define FLASH_PAGE_PROG_TIME                6
#define FLASH_READ_QWORD                    1
#define FLASH_READ_DWORD                    1
#define FLASH_READ_WORD                     1
#define FLASH_READ_BYTE                     1
#define FLASH_PROG_QWORD                    1
#define FLASH_PROG_DWORD                    0
#define FLASH_PROG_WORD                     0
#define FLASH_PROG_BYTE                     0

#define FLASH_TOTAL_SIZE                    (FLASH_ADDR_END-FLASH_ADDR_START+1)
#define FLASH_BLDR_SIZE                     (FLASH_ADDR_BLDR_END-FLASH_ADDR_BLDR_START+1)
#define FLASH_VIRTUAL_EEPROM_SIZE           (FLASH_ADDR_VIRTUAL_EEPROM_END-FLASH_ADDR_VIRTUAL_EEPROM_START+1)
#define FLASH_APPCODE_SIZE                  (FLASH_ADDR_APPCODE_END-FLASH_ADDR_APPCODE_START+1)

#define EEPROM_TOTAL_SIZE                   FLASH_VIRTUAL_EEPROM_SIZE
#define EEPROM_PAGE_NUM                     1
#define EEPROM_PAGE_ERASE_TIME              4
#define EEPROM_PAGE_PROG_TIME               6

#define EEPROM_ADDR_COMMIT_IMG_SIGNATURE   (EEPROM_TOTAL_SIZE-4)
/* Exported types ------------------------------------------------------------*/
#if (BL_USE_STDLIB == 0)
typedef unsigned char                       uint8_t;
typedef unsigned int                        uint16_t;
typedef unsigned long int                   uint32_t;
typedef char                                int8_t;
typedef int                                 int16_t;
typedef long int                            int32_t;
typedef uint32_t                            size_t;

extern void *memcpy(void *_Restrict, const void *_Restrict, size_t);
extern void *memset(void *, int, size_t);
extern int strncmp(const char *, const char *, size_t);
#endif
/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* __BL_CONFIG_H */

/******************************** END OF FILE *********************************/

