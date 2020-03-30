/*******************************************************************************
 * Copyright (c) 2020-2021, Single-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Change Logs:
 * Date         Author       Notes
 * 2020-03-25   Wentao SUN   first version
 *
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_FLASH_H
#define __HAL_FLASH_H

/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define HAL_FLASH_APP_CODE_START_ADDR       0x08002000                        // start flash address for app-code
#define HAL_FLASH_APP_CODE_NUM_OF_PAGE      192                               // number of flash pages (24KB) for app-code
#define HAL_FLASH_PAGE_SIZE                 128                               // 128-byte per page
#define HAL_FLASH_CELL_WIDTH                4                                 // 4-byte cell width
#define HAL_FLASH_PAGE_ERASE_TIME           10                                // erase time of a page

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_flash_erase_page( uint16_t page_id );
void hal_flash_write( uint32_t addr, uint8_t *pdata, uint16_t len );
uint8_t hal_flash_read( uint32_t addr );


#endif /* __HAL_FLASH_H */

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/

