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
 
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "hal_mcu.h"
#include "hal_uart.h"
#include "hal_flash.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t cmd[256];
static uint16_t rxcnt;
static uint16_t txcnt;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void main( void )
{
    hal_uart_config_t cfg;
    
    hal_mcu_init();
    //hal_mcu_wdg_start();
    
    cfg.baud_rate = HAL_UART_BAUD_RATE_115200;
    cfg.bit_order = HAL_UART_BIT_ORDER_LSB;
    cfg.data_bits = HAL_UART_DATA_BITS_8;
    cfg.invert = HAL_UART_NRZ_NORMAL;
    cfg.parity = HAL_UART_PARITY_NONE;
    cfg.stop_bits = HAL_UART_STOP_BITS_1;
    
    hal_uart_init(HAL_UART_PORT_0, &cfg);
    hal_uart_open(HAL_UART_PORT_0);

    rxcnt = 0;
    for(;;)
    {
        //hal_mcu_wdg_reset();

        rxcnt = hal_uart_read(HAL_UART_PORT_0, cmd, sizeof(cmd));
        if(rxcnt)
        {
            txcnt = 0;
            while(txcnt < rxcnt)
              txcnt += hal_uart_write(HAL_UART_PORT_0, cmd+txcnt, rxcnt-txcnt);
        }
    }

}
/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/
