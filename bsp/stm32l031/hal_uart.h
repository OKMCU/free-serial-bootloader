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
enum {
    HAL_UART_PORT_0 = 0,
    HAL_UART_PORT_MAX
};

#define HAL_UART_BAUD_RATE_2400                  2400
#define HAL_UART_BAUD_RATE_4800                  4800
#define HAL_UART_BAUD_RATE_9600                  9600
#define HAL_UART_BAUD_RATE_19200                 19200
#define HAL_UART_BAUD_RATE_38400                 38400
#define HAL_UART_BAUD_RATE_57600                 57600
#define HAL_UART_BAUD_RATE_115200                115200
#define HAL_UART_BAUD_RATE_230400                230400
#define HAL_UART_BAUD_RATE_256000                256000
#define HAL_UART_BAUD_RATE_460800                460800
#define HAL_UART_BAUD_RATE_921600                921600
#define HAL_UART_BAUD_RATE_2000000               2000000
#define HAL_UART_BAUD_RATE_3000000               3000000

#define HAL_UART_DATA_BITS_5                     5
#define HAL_UART_DATA_BITS_6                     6
#define HAL_UART_DATA_BITS_7                     7
#define HAL_UART_DATA_BITS_8                     8
#define HAL_UART_DATA_BITS_9                     9

#define HAL_UART_STOP_BITS_1                     0
#define HAL_UART_STOP_BITS_2                     1
#define HAL_UART_STOP_BITS_3                     2
#define HAL_UART_STOP_BITS_4                     3

#define HAL_UART_PARITY_NONE                     0
#define HAL_UART_PARITY_ODD                      1
#define HAL_UART_PARITY_EVEN                     2

#define HAL_UART_BIT_ORDER_LSB                   0
#define HAL_UART_BIT_ORDER_MSB                   1

#define HAL_UART_NRZ_NORMAL                      0     /* normal mode */
#define HAL_UART_NRZ_INVERTED                    1     /* inverted mode */

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint32_t baud_rate;
    uint16_t data_bits   :4;
    uint16_t stop_bits   :2;
    uint16_t parity      :2;
    uint16_t bit_order   :1;
    uint16_t invert      :1;
    uint16_t reserved    :6;
} hal_uart_config_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_uart_init(uint8_t port, const hal_uart_config_t *cfg);
void hal_uart_open(uint8_t port);
uint16_t hal_uart_write(uint8_t port, const uint8_t *pbuf, uint16_t len);
uint16_t hal_uart_read(uint8_t port, uint8_t *pbuf, uint16_t size);
void hal_uart_close(uint8_t port);
void hal_uart_deinit(uint8_t port);

#endif /* __HAL_UART_H */

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/

