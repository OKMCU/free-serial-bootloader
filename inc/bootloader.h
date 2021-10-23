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
#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H
/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Check commit image signature in (virtual) EEPROM. 
  *         Jump to APPCODE image if it exists and is committed.
  * @note   Call this function at the beginning of the main() function or 
  *         in reset handler routine.
  * @param  none
  * @retval none
  */
void bootloader_reset_handler(void);

/**
  * @brief  Bootloader service.
  * @note   There's a dead loop in this function and it would never return.
  * @param  none
  * @retval none
  */
void bootloader_service(void)

#endif /* __BOOTLOADER_H */

/******************************** END OF FILE *********************************/

