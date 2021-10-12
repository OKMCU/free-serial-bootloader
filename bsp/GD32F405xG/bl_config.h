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
#define BLDR_PROG_BUILD_TIME                "2021-10-12 19:15:58"
#define BLDR_PROG_MAJOR_VER                 0
#define BLDR_PROG_MINOR_VER                 0
#define BLDR_PROG_BUILD_VER                 1
#define BLDR_PROG_FLASH_ADDR                0x08000000UL
#define BLDR_PROG_SIZE                      0x00004000UL
#define APPCODE_PTR_FLASH_SECTOR_ID         11
#define CHIP_FLASH_START_ADDR               0x08000000UL
#define CHIP_FLASH_SIZE                     0x00100000UL
#define CHIP_FLASH_SECTOR_NUM               12
#define CHIP_FLASH_MASS_ERASE_TIME          30000
#define CHIP_FLASH_MIN_WR_BLK_SIZE          1
#define CHIP_FLASH_MAX_WR_BLK_TIME          1
#define CHIP_EEPROM_START_ADDR              0x08040000UL
#define CHIP_EEPROM_SIZE                    0x00020000UL
#define CHIP_EEPROM_SECTOR_NUM              1
#define CHIP_EEPROM_MASS_ERASE_TIME         8000
#define CHIP_EEPROM_MIN_WR_BLK_SIZE         1
#define CHIP_EEPROM_MAX_WR_BLK_TIME         1
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

