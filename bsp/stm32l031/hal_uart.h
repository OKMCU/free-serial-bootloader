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
#ifndef __HAL_UART_H
#define __HAL_UART_H

/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_uart_init(void);
void hal_uart_open(uint32_t baudrate);
uint16_t hal_uart_write(const uint8_t *pbuf, uint16_t len);
uint16_t hal_uart_read(uint8_t *pbuf, uint16_t size);
void hal_uart_close(void);
#endif /* __HAL_UART_H */

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/

