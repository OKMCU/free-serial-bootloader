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
    uint16_t i;
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

    for(i = 0; i < HAL_FLASH_PAGE_SIZE; i++)
    {
        cmd[i] = hal_flash_read(0x08000000+i);
    }
    txcnt = 0;
    while(txcnt < HAL_FLASH_PAGE_SIZE)
        txcnt += hal_uart_write(HAL_UART_PORT_0, cmd+txcnt, HAL_FLASH_PAGE_SIZE-txcnt);
    rxcnt = 0;
    for(;;)
    {
        //hal_mcu_wdg_reset();

        rxcnt = hal_uart_read(HAL_UART_PORT_0, cmd, sizeof(cmd));
        if(rxcnt)
        {
            //txcnt = 0;
            //while(txcnt < rxcnt)
            //    txcnt += hal_uart_write(HAL_UART_PORT_0, cmd+txcnt, rxcnt-txcnt);
            
            switch(cmd[0])
            {
                case 0x00:
                    for(i = 0; i < HAL_FLASH_PAGE_SIZE; i++)
                    {
                        cmd[i] = i;
                    }
                    hal_flash_write(HAL_FLASH_APP_CODE_START_ADDR, cmd, HAL_FLASH_PAGE_SIZE);
                break;
                
                case 0x01:
                    for(i = 0; i < HAL_FLASH_PAGE_SIZE; i++)
                    {
                        cmd[i] = hal_flash_read(HAL_FLASH_APP_CODE_START_ADDR+i);
                    }
                    txcnt = 0;
                    while(txcnt < HAL_FLASH_PAGE_SIZE)
                        txcnt += hal_uart_write(HAL_UART_PORT_0, cmd+txcnt, HAL_FLASH_PAGE_SIZE-txcnt);
                break;
                
                case 0xFF:
                    hal_flash_erase_page(HAL_FLASH_APP_CODE_START_ADDR);
                break;
            }
        }

        
    }

}
/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/
