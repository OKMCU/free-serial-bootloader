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
 
/* Includes ------------------------------------------------------------------*/
#include "hal.h"
/* Private define ------------------------------------------------------------*/
#define HAL_UART_BUF_SIZE                   1024
#define HST_HANDSHAKING_BYTE                0x55
#define DEV_HANDSHAKING_BYTE                0xAA
#define HANDSHAKING_SUCCESS_THRE            16
#define HST_PACKET_START_BYTE               0xAA
#define DEV_PACKET_START_BYTE               0x55

#define PACKET_START_INDEX                  0
#define PACKET_LENGTH_INDEX                 1
#define PACKET_PAYLOAD_START_INDEX          2
#define PACKET_PAYLOAD_TYPE_INDEX           (PACKET_PAYLOAD_START_INDEX+0)
#define PACKET_PAYLOAD_REGADDR_INDEX        (PACKET_PAYLOAD_START_INDEX+1)

#define PAYLOAD_TYPE_SET                    0x00
#define PAYLOAD_TYPE_GET                    0x01

#define STS_SUCCESS                         0x00
#define STS_FAILURE_GENERIC                 (0x80|0x00)
#define STS_FAILURE_WRONG_CRC               (0x80|0x01)
#define STS_FAILURE_WRONG_TYPE              (0x80|0x02)

#define REG_ADDR_STATUS                     0x0E
/* Private typedef -----------------------------------------------------------*/
typedef struct uart_rx_buffer {
    uint8_t data[HAL_UART_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} uart_rx_buffer_t;

typedef struct payload {
    uint8_t type;
    uint8_t reg_addr;
    uint8_t size;
    uint8_t *reg_data;
} payload_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uart_rx_buffer_t rb;
static uint8_t packet_buffer[256];
static uint8_t packet_rxcnt = 0;
static uint8_t handshaking_cnt = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void on_uart_recv_byte(uint8_t rx_byte)
{
    uint16_t curr_head, curr_tail, next_tail;

    curr_head = rb.head;
    curr_tail = rb.tail;
    next_tail = curr_tail+1;
    if(next_tail >= HAL_UART_BUF_SIZE) next_tail = 0;
    if(next_tail != curr_head)
    {
        rb.data[curr_tail] = rx_byte;
        rb.tail = next_tail;
    }
}

static uint8_t calc_crc8(uint8_t crc, uint8_t *p_buf, uint32_t size)
{
    return 0x00;
}

static void send_packet_reg_set(uint8_t reg_addr, uint8_t *reg_data, uint8_t size)
{
    uint8_t b[2], i, crc = 0x00;
    if(size > 128) size = 128;
    b[0] = PAYLOAD_TYPE_SET;
    b[1] = reg_addr;
    crc = calc_crc8(crc, b, 2);
    crc = calc_crc8(crc, reg_data, size);
    hal_uart_send(DEV_PACKET_START_BYTE);
    hal_uart_send(2+size);
    hal_uart_send(PAYLOAD_TYPE_SET);
    hal_uart_send(reg_addr);
    for(i = 0; i < size; i++)
    {
        hal_uart_send(reg_data[i]);
    }
    hal_uart_send(crc);
}

static void on_recv_packet(const uint8_t *pkt, uint8_t pkt_length)
{
    uint8_t crc;
    uint8_t *payload;
    uint8_t payload_length;
    payload = pkt+PACKET_PAYLOAD_START_INDEX;
    payload_length = pkt[PACKET_LENGTH_INDEX];
    crc = pkt[pkt_length-1];

    if(crc == calc_crc8(0x00, payload, payload_length))
    {
        if(pkt[PACKET_PAYLOAD_TYPE_INDEX] == PAYLOAD_TYPE_SET)
        {
            send_packet_reg_sts_payload(pkt[PACKET_PAYLOAD_REGADDR_INDEX], STS_BUSY_CMD_CAPTURED);
        }
        else if(pkt[PACKET_PAYLOAD_TYPE_INDEX] == PAYLOAD_TYPE_GET)
        {
        }
        else
        {
            send_packet_reg_sts_payload(pkt[PACKET_PAYLOAD_REGADDR_INDEX], STS_FAIL_PARAM_NOT_SUPPORTED);
        }
    }
    else
    {
        send_packet_reg_set(REG_ADDR_STATUS, STS_FAILURE_WRONG_CRC);
    }
}

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void bootloader_reset_handler(void)
{
    uint32_t bldr_signature;
    uint32_t appl;
    /* read bootloader signature to check if APPCODE has been commited */
    if(hal_eep_read(EEPROM_ADDR_BLDR_SIGNATURE, &bldr_signature, 4) == HAL_OK)
    {
        if(bldr_signature == BLDR_SIGNATURE)
        {
            /* read first 4 bytes at starting address of APPCODE */
            if(hal_fmc_read(FLASH_ADDR_APPCODE_START, &appl, 4) == HAL_OK)
            {
                /* confirm APPCODE exists */
                if(appl != 0xFFFFFFFF)
                {
                    /* jump to APPCODE */
                    hal_jump_to_addr(FLASH_ADDR_APPCODE_START);
                }
            }
        }
    }
}

void bootloader_service(void)
{
    uint16_t head, tail, rx_byte;
    hal_enter_critical();
    hal_board_init();
    hal_wdg_config(HAL_WDG_TIMEOUT);
    hal_wdg_start();
    hal_uart_config(HAL_UART_BAUDRATE, on_uart_recv_byte);
    hal_exit_critical();

    while(1)
    {
        /* refresh watch dog counter */
        hal_wdg_refresh();

        /* check uart rx buffer */
        hal_enter_critical();
        head = rb.head;
        tail = rb.tail;
        hal_exit_critical();

        /* process rx data */
        while(head != tail)
        {
            rx_byte = rb.data[head++];
            if(head >= HAL_UART_BUF_SIZE) head = 0;

            if(packet_rxcnt == PACKET_START_INDEX)
            {
                if(rx_byte == HST_PACKET_START_BYTE)
                {
                    packet_buffer[packet_rxcnt++] = rx_byte;
                }
            }
            else if(packet_rxcnt == PACKET_LENGTH_INDEX)
            {
                packet_buffer[packet_rxcnt++] = rx_byte;
                if(rx_byte > 128+2 || rx_byte < 2)
                {
                    packet_rxcnt = 0;
                }
            }
            else
            {
                packet_buffer[packet_rxcnt++] = rx_byte;
                if(packet_rxcnt >= packet_buffer[PACKET_LENGTH_INDEX]+3)
                {
                    on_recv_packet(packet_buffer, packet_rxcnt);
                }
            }

            if(packet_rxcnt == PACKET_START_INDEX)
            {
                if(rx_byte == HST_HANDSHAKING_BYTE)
                {
                    if(handshaking_cnt < UINT8_MAX) handshaking_cnt++;
                    if(handshaking_cnt >= HANDSHAKING_SUCCESS_THRE)
                    {
                        /* handshaking success */
                        hal_uart_send(DEV_HANDSHAKING_BYTE);
                    }
                }
                else
                {
                    handshaking_cnt = 0;
                }
            }
        }

        /* update head position of the ring buffer */
        hal_enter_critical();
        rb.head = head;
        hal_exit_critical();
    }
}
/******************************** END OF FILE *********************************/
