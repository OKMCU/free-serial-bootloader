/*******************************************************************************
 * Copyright (c) 2021-2022, OKMCU Development Team 
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Website: http://www.okmcu.com
 *
 * File Description: CRC algotrithm library
 *
 * Change Logs:
 * Date         Author       Notes    
 * 2021-10-24   Wentao SUN   first version
 * 
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRC_H
#define __CRC_H
/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  CRC8/maxim calculation with initial crc-8 value.
  * @param  crc [I] - initial crc-8 value
  * @param  pdata [I] - points to data block
  * @param  size [I] - size of the data block
  * @retval updated crc-8
  */
uint8_t crc8_maxim_update(uint8_t crc, const uint8_t * pdata, size_t size);

/**
  * @brief  CRC8/maxim calculation starting from 0x00.
  * @param  pdata [I] - points to data block
  * @param  size [I] - size of the data block
  * @retval calculated crc-8
  */
uint8_t crc8_maxim(const uint8_t * pdata, size_t size);

#endif /* __CRC_H */

/******************************** END OF FILE *********************************/

