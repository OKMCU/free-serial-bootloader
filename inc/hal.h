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
#ifndef __HAL_H
#define __HAL_H
/* Includes ------------------------------------------------------------------*/
#include "bl_config.h"
/* Exported constants --------------------------------------------------------*/
#define HAL_OK                  0
#define HAL_ERR                 -1
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Enable global interrupt.
  * @param  none
  * @retval none
  */
void hal_enter_critical(void);

/**
  * @brief  Disable global interrupt.
  * @param  none
  * @retval none
  */
void hal_exit_critical(void);

/**
  * @brief  MCU software reset.
  * @param  none
  * @retval none
  */
void hal_reset(void);

/**
  * @brief  Setup stack pointer and jump to APROM reset vector.
  * @param  addr [I] - FLASH address of APROM start.
  * @retval none
  * @note   No return from this function.
  */
void hal_jump_to_addr(uint32_t addr);

/**
  * @brief  Read MCU universal unique ID in ASCII format.
  * @param  p_uuid [I] - buffer to receive UUID
  * @param  size [I] - buffer size
  * @retval string length in the buffer
  */
uint8_t hal_mcu_get_uuid(char *p_uuid, uint8_t size);

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
/**
  * @brief  Free watch dog timeout counter configuration.
  * @param  timeout [I] - timeout threshold in millisecond
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_wdg_config(uint16_t timeout);

/**
  * @brief  Free watch dog timeout counter start.
  * @param  none
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_wdg_start(void);

/**
  * @brief  Free watch dog timeout counter refresh.
  * @param  none
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_wdg_refresh(void);
#endif //defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)

/**
  * @brief  Read FLASH page information.
  * @param  page [I] - FLASH page ID
  * @param  addr [O] - FLASH page starting address
  * @param  size [O] - FLASH page size (number of bytes)
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_flash_page_info(int32_t page, uint32_t *addr, uint32_t *size);

/**
  * @brief  Erase a FLASH page.
  * @param  page [I] - FLASH page ID
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_flash_page_erase(int32_t page);

/**
  * @brief  Read data from FLASH.
  * @param  addr [I] - starting FLASH address to read
  * @param  p_buf [O] - buffer to receive FLASH data
  * @param  size [O] - number of bytes to read
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_flash_read(uint32_t addr, void *p_buf, uint32_t size);

/**
  * @brief  Write data to FLASH.
  * @param  addr [I] - starting FLASH address to read
  * @param  p_buf [I] - points to data to be written into FLASH
  * @param  size [I] - number of bytes to write
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_flash_write(uint32_t addr, const void *p_buf, uint32_t size);

/**
  * @brief  Read EEPROM page information.
  * @param  page [I] - EEPROM page ID
  * @param  addr [O] - EEPROM page starting address
  * @param  size [O] - EEPROM page size (number of bytes)
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_eeprom_page_info(int32_t page, uint32_t *addr, uint32_t *size);

/**
  * @brief  Erase a EEPROM page.
  * @param  page [I] - EEPROM page ID
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_eeprom_page_erase(int32_t page);

/**
  * @brief  Read data from EEPROM.
  * @param  addr [I] - starting EEPROM address to read
  * @param  p_buf [O] - buffer to receive EEPROM data
  * @param  size [O] - number of bytes to read
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_eeprom_read(uint32_t addr, void *p_buf, uint32_t size);

/**
  * @brief  Write data to EEPROM.
  * @param  addr [I] - starting EEPROM address to read
  * @param  p_buf [I] - points to data to be written into EEPROM
  * @param  size [I] - number of bytes to write
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_eeprom_write(uint32_t addr, const void *p_buf, uint32_t size);

/**
  * @brief  UART baud rate configuration.
  * @param  baudrate [I] - baudrate
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_uart_config(uint32_t baudrate);

/**
  * @brief  Send data through UART interface. This function sends data in blocking mode.
  * @param  p_data [I] - points to data to send
  * @param  txlen [I] - number of bytes to send
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  * @note   
  */
int8_t hal_uart_send(const uint8_t *p_data, uint32_t txlen);

/**
  * @brief  Receive data through UART interface. This function receives data in blocking mode.
  *         It never returns until the buffer is full or an idle frame is detected on UART RXD line.
  * @param  p_data [I] - buffer to receive data
  * @param  size [I] - buffer size
  * @param  rxlen [O] - number of bytes received
  * @retval HAL operation result
  *         @ref HAL_OK
  *         @ref HAL_ERR
  */
int8_t hal_uart_recv(uint8_t *p_data, uint32_t size, uint32_t *rxlen);

#endif /* __HAL_H */

/******************************** END OF FILE *********************************/

