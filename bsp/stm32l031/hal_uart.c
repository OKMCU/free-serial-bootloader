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
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CPU_APB1CLK                 (32000000uL)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_uart_init(void)
{
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART2 );
    LL_APB1_GRP1_ForceReset( LL_APB1_GRP1_PERIPH_USART2 );
    LL_APB1_GRP1_ReleaseReset( LL_APB1_GRP1_PERIPH_USART2 );
    LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP );
    LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_UP );
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE ); // PA2:  USART2_TX
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE );// PA15: USART2_RX
    LL_GPIO_SetAFPin_0_7( GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7 );         // PA2:  USART2_TX
    LL_GPIO_SetAFPin_0_7( GPIOA, LL_GPIO_PIN_15, LL_GPIO_AF_7 );        // PA15: USART2_RX
    NVIC_EnableIRQ( USART2_IRQn );
}

void hal_uart_open(uint32_t baudrate)
{
    LL_USART_SetBaudRate( USART2, CPU_APB1CLK, LL_USART_OVERSAMPLING_16, baudrate );
    // set data bits
    LL_USART_SetDataWidth( USART2, LL_USART_DATAWIDTH_8B );

    // set stop bits
    LL_USART_SetStopBitsLength( USART2, LL_USART_STOPBITS_1 );

    // set parity
    LL_USART_SetParity( USART2, LL_USART_PARITY_NONE );

    // set bit order
    LL_USART_SetTransferBitOrder( USART2, LL_USART_BITORDER_LSBFIRST );

    // set data invert
    LL_USART_SetBinaryDataLogic( USART2, LL_USART_BINARY_LOGIC_POSITIVE );

    LL_USART_EnableDirectionRx( USART2 );
    LL_USART_EnableDirectionTx( USART2 );
    LL_USART_EnableIT_RXNE( USART2 );
    LL_USART_EnableIT_IDLE( USART2 );
    LL_USART_Enable( USART2 );

}

uint16_t hal_uart_write(const uint8_t *pbuf, uint16_t len)
{
    return 0;
}

uint16_t hal_uart_read(uint8_t *pbuf, uint16_t size)
{
    return 0;
}

void hal_uart_close(void)
{
    LL_USART_Disable( USART2 );
}

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/
