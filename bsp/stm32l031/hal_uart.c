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
#include <string.h>
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_usart.h"
#include "hal_uart.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t *rx_cache;
    uint8_t *tx_cache;
    uint8_t rx_cache_size;
    uint8_t tx_cache_size;
} uart_cache_t;

typedef struct {
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t tx_head;
    uint8_t tx_tail;
} uart_ctrl_t;

/* Private define ------------------------------------------------------------*/
#define UART0_RX_CACHE_SIZE         8
#define UART0_TX_CACHE_SIZE         8
#define CPU_APB1CLK                 (32000000uL)
/* Private macro -------------------------------------------------------------*/
#define RING_BUF_INC_INDEX( index, size )           index = (index+1)>=size ? 0:(index+1)
#define RING_BUF_FLUSH( head, tail )                do{ head = 0; tail = 0; }while(0)
#define RING_BUF_PUT( byte, head, p_buf, size )     do{ p_buf[head] = byte; RING_BUF_INC_INDEX(head, size); }while(0)
#define RING_BUF_GET( p_byte, tail, p_buf, size )   do{ *(p_byte) = p_buf[tail]; RING_BUF_INC_INDEX(tail, size); }while(0)
#define RING_BUF_EMPTY( head, tail )                (head==tail)
#define RING_BUF_FULL( head, tail, size )           (((head+1)>=size?0:(head+1))==tail)
#define RING_BUF_USED_SIZE( head, tail, size )      ((head>=tail)?(head-tail):(size-(tail-head)))
#define RING_BUF_FREE_SIZE( head, tail, size )      ((head>=tail)?(size-1-(head-tail)):(tail-head-1))

/* Private variables ---------------------------------------------------------*/
static uint8_t uart0_rx_cache[UART0_RX_CACHE_SIZE];
static uint8_t uart0_tx_cache[UART0_TX_CACHE_SIZE];
static uart_cache_t const uart_cache[HAL_UART_PORT_MAX] = {
    { 
        .rx_cache = uart0_rx_cache, 
        .tx_cache = uart0_tx_cache, 
        .rx_cache_size = sizeof(uart0_rx_cache), 
        .tx_cache_size = sizeof(uart0_tx_cache),
    }
};

static USART_TypeDef* const USARTx[HAL_UART_PORT_MAX] = {
    USART2
};

static uint32_t const PeriphClk[HAL_UART_PORT_MAX] = {
    CPU_APB1CLK
};

static uart_ctrl_t uart_ctrl[HAL_UART_PORT_MAX];
/* Private function prototypes -----------------------------------------------*/
static void hal_uart_isr( uint8_t port );
extern void USART1_IRQHandler( void );
extern void USART2_IRQHandler( void );
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_uart_init(uint8_t port, const hal_uart_config_t *cfg)
{
    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    //ST_ASSERT( cfg != NULL );
    
    // reset peripherals firstly
    switch ( port )
    {
        case HAL_UART_PORT_0:
            LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART2 );
            LL_APB1_GRP1_ForceReset( LL_APB1_GRP1_PERIPH_USART2 );
            LL_APB1_GRP1_ReleaseReset( LL_APB1_GRP1_PERIPH_USART2 );
            LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP );
            LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_UP );
            //LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE ); // PA2:  USART2_TX
            //LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE );// PA15: USART2_RX
            LL_GPIO_SetAFPin_0_7( GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7 );         // PA2:  USART2_TX
            LL_GPIO_SetAFPin_0_7( GPIOA, LL_GPIO_PIN_15, LL_GPIO_AF_4 );        // PA15: USART2_RX
            NVIC_EnableIRQ( USART2_IRQn );
        break;
    }

    // set baud rate
    LL_USART_SetBaudRate( USARTx[port],
                          PeriphClk[port],
                          LL_USART_OVERSAMPLING_16, 
                          cfg->baud_rate );

    // set data bits
    switch ( cfg->data_bits )
    {
        //case HAL_UART_DATA_BITS_7:
        //    LL_USART_SetDataWidth( USARTx[port], LL_USART_DATAWIDTH_7B );
        //break;
        //case HAL_UART_DATA_BITS_8:
        //    LL_USART_SetDataWidth( USARTx[port], LL_USART_DATAWIDTH_8B );
        //break;
        //case HAL_UART_DATA_BITS_9:
        //    LL_USART_SetDataWidth( USARTx[port], LL_USART_DATAWIDTH_9B );
        //break;
        default:
            LL_USART_SetDataWidth( USARTx[port], LL_USART_DATAWIDTH_8B );
        break;
    }

    // set stop bits
    switch ( cfg->stop_bits )
    {
        case HAL_UART_STOP_BITS_1:
            LL_USART_SetStopBitsLength( USARTx[port], LL_USART_STOPBITS_1 );
        break;

        case HAL_UART_STOP_BITS_2:
            LL_USART_SetStopBitsLength( USARTx[port], LL_USART_STOPBITS_2 );
        break;

        default:
            LL_USART_SetStopBitsLength( USARTx[port], LL_USART_STOPBITS_1 );
        break;
    }

    // set parity
    switch ( cfg->parity )
    {
        case HAL_UART_PARITY_EVEN:
            LL_USART_SetParity( USARTx[port], LL_USART_PARITY_EVEN );
            LL_USART_EnableIT_PE( USARTx[port] );
        break;
        case HAL_UART_PARITY_ODD:
            LL_USART_SetParity( USARTx[port], LL_USART_PARITY_ODD );
            LL_USART_EnableIT_PE( USARTx[port] );
        break;
        default:
            LL_USART_SetParity( USARTx[port], LL_USART_PARITY_NONE );
        break;
    }

    // set bit order
    if( cfg->bit_order == HAL_UART_BIT_ORDER_LSB )
        LL_USART_SetTransferBitOrder( USARTx[port], LL_USART_BITORDER_LSBFIRST );
    else
        LL_USART_SetTransferBitOrder( USARTx[port], LL_USART_BITORDER_MSBFIRST );

    // set data invert
    if( cfg->invert == HAL_UART_NRZ_NORMAL )
        LL_USART_SetBinaryDataLogic( USARTx[port] , LL_USART_BINARY_LOGIC_POSITIVE );
    else
        LL_USART_SetBinaryDataLogic( USARTx[port] , LL_USART_BINARY_LOGIC_NEGATIVE );

    // init uart control body info
    memset( &uart_ctrl[port], 0, sizeof(uart_ctrl_t) );
    
    LL_USART_EnableDirectionRx( USARTx[port] );
    LL_USART_EnableDirectionTx( USARTx[port] );
    LL_USART_EnableIT_RXNE( USARTx[port] );
    LL_USART_EnableIT_IDLE( USARTx[port] );
    //LL_USART_Enable( USARTx[port] );
}

void hal_uart_open(uint8_t port)
{
    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    //ST_ASSERT( !LL_USART_IsEnabled(USARTx[port]) );
    LL_USART_Enable( USARTx[port] );
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE ); // PA2:  USART2_TX
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE );// PA15: USART2_RX
}

uint16_t hal_uart_write(uint8_t port, const uint8_t *pbuf, uint16_t len)
{
    uint16_t cnt;
    uint8_t byte;
    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    //ST_ASSERT( LL_USART_IsEnabled(USARTx[port]) );
    
    for(cnt = 0; cnt < len; cnt++)
    {
        if( RING_BUF_FULL(uart_ctrl[port].tx_head, 
                          uart_ctrl[port].tx_tail, 
                          uart_cache[port].tx_cache_size) )
        {
            break;
        }
        
        if( RING_BUF_EMPTY(uart_ctrl[port].tx_head, uart_ctrl[port].tx_tail) &&
            LL_USART_IsActiveFlag_TXE(USARTx[port]) )
        {
            LL_USART_TransmitData8( USARTx[port], byte );
            LL_USART_EnableIT_TXE( USARTx[port] );
        }
        else
        {
            LL_USART_DisableIT_TXE( USARTx[port] );
            RING_BUF_PUT( byte,
                          uart_ctrl[port].tx_head, 
                          uart_cache[port].tx_cache, 
                          uart_cache[port].tx_cache_size );
            LL_USART_EnableIT_TXE( USARTx[port] );
        }
    }
    
    return cnt;
}

uint16_t hal_uart_read(uint8_t port, uint8_t *pbuf, uint16_t size)
{
    uint16_t cnt;
    uint8_t byte;
    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    //ST_ASSERT( LL_USART_IsEnabled(USARTx[port]) );
    
    for(cnt = 0; cnt < size; cnt++)
    {
        if( RING_BUF_EMPTY(uart_ctrl[port].rx_head, 
                           uart_ctrl[port].rx_tail) )
        {
            break;
        }
        
        LL_USART_DisableIT_RXNE( USARTx[port] );
        RING_BUF_GET( &byte, 
                      uart_ctrl[port].rx_tail, 
                      uart_cache[port].rx_cache,
                      uart_cache[port].rx_cache_size );
        LL_USART_EnableIT_RXNE( USARTx[port] );
        pbuf[cnt] = byte;
    }
    
    return cnt;
}

void hal_uart_close(uint8_t port)
{
    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    //ST_ASSERT( LL_USART_IsEnabled(USARTx[port]) );
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG ); // PA2:  USART2_TX
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ANALOG );// PA15: USART2_RX
    LL_USART_Disable( USARTx[port] );
}

void hal_uart_deinit(uint8_t port)
{
    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    //ST_ASSERT( LL_USART_IsEnabled(USARTx[port]) );
    
    LL_USART_DisableDirectionTx( USARTx[port] );
    LL_USART_DisableDirectionRx( USARTx[port] );
    LL_USART_DisableIT_RXNE( USARTx[port] );
    LL_USART_DisableIT_IDLE( USARTx[port] );
    LL_USART_Disable( USARTx[port] );

    switch ( port )
    {
        case HAL_UART_PORT_0:
            NVIC_DisableIRQ( USART2_IRQn );
            LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG );   // PA2: USART2_TX
            LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ANALOG );  // PA15: USART2_RX
            LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO );
            LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_NO );
            LL_APB1_GRP1_DisableClock( LL_APB1_GRP1_PERIPH_USART2 );
        break;
    }
}


static void hal_uart_isr( uint8_t port )
{
    uint8_t byte;

    //ST_ASSERT( port < HAL_UART_PORT_MAX );
    
    if( LL_USART_IsActiveFlag_RXNE(USARTx[port]) )
    {
        byte = LL_USART_ReceiveData8( USARTx[port] );
        if( RING_BUF_FULL(uart_ctrl[port].rx_head, 
                          uart_ctrl[port].rx_tail, 
                          uart_cache[port].rx_cache_size) )
        {
            //st_task_set_event( task_id_rxd, uart_event[port].ovf );
        }
        else
        {
            RING_BUF_PUT( byte, 
                          uart_ctrl[port].rx_head, 
                          uart_cache[port].rx_cache, 
                          uart_cache[port].rx_cache_size );
            //st_task_set_event( task_id_rxd, uart_event[port].rxd );
        }
        return;
    }

    if( LL_USART_IsActiveFlag_PE(USARTx[port]) )
    {
        //st_task_set_event( task_id_rxd, uart_event[port].perr );
        LL_USART_ClearFlag_PE( USARTx[port] );
        return;
    }

    if( LL_USART_IsActiveFlag_IDLE( USARTx[port]) )
    {
        //st_task_set_event( task_id_rxd, uart_event[port].idle );
        LL_USART_ClearFlag_IDLE( USARTx[port] );
        return;
    }
    
    if( LL_USART_IsActiveFlag_TXE(USARTx[port]) )
    {
        if( RING_BUF_EMPTY(uart_ctrl[port].tx_head, uart_ctrl[port].tx_tail) )
        {
            LL_USART_DisableIT_TXE( USARTx[port] );
        }
        else
        {
            RING_BUF_GET( &byte, 
                          uart_ctrl[port].tx_tail, 
                          uart_cache[port].tx_cache, 
                          uart_cache[port].tx_cache_size );
            LL_USART_TransmitData8( USARTx[port], byte );
            //st_task_set_event( task_id_txd, uart_event[port].txd );
        }
        return;
    }
}

void USART2_IRQHandler( void )
{
    hal_uart_isr( HAL_UART_PORT_0 );
}
/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/
