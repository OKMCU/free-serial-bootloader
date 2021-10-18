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

#define PKT_HDR_IDX_START                   0
#define PKT_HDR_IDX_CRC8                    1
#define PKT_HDR_IDX_TYPE                    2
#define PKT_HDR_IDX_REGADDR                 3
#define PKT_HDR_IDX_STATUS                  4
#define PKT_HDR_IDX_LENGTH                  5
#define PKT_HDR_SIZE                        6

#define TYPE_SET                            0x01
#define TYPE_GET                            0x02

#define STS_NA                              0x00
#define STS_SUCCESS                         0x01
#define STS_FAILURE_UNKNOWN_REG             (0x80|0x00)

/* Private typedef -----------------------------------------------------------*/
typedef struct uart_rx_buffer_s {
    uint8_t data[HAL_UART_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} uart_rx_buffer_t;

typedef union packet_s {
    uint8_t all[PKT_HDR_SIZE+128];
    struct {
        struct {
            uint8_t start;
            uint8_t crc8;
            uint8_t type;
            uint8_t reg_addr;
            uint8_t status;
            uint8_t length;
        } header;
        uint8_t payload[128];
    } part;
} packet_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uart_rx_buffer_t rb;
static uint8_t packet_rxcnt = 0;
static uint8_t handshaking_cnt = 0;
static packet_t packet;
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

static int8_t send_packet(uint8_t type, uint8_t reg_addr, uint8_t status, uint8_t length, uint8_t *payload)
{
    uint8_t i, crc = 0x00;
    if(length > 128) return -1;
    if(type != TYPE_SET && type!= TYPE_GET) return -1;

    crc = calc_crc8(crc, &type, 1);
    crc = calc_crc8(crc, &reg_addr, 1);
    crc = calc_crc8(crc, &status, 1);
    crc = calc_crc8(crc, &length, 1);
    crc = calc_crc8(crc, &payload, length);
    hal_uart_send(DEV_PACKET_START_BYTE);
    hal_uart_send(crc);
    hal_uart_send(type);
    hal_uart_send(reg_addr);
    hal_uart_send(status);
    hal_uart_send(length);
    for(i = 0; i < length; i++)
    {
        hal_uart_send(payload[i]);
    }
}

static void on_recv_packet(const packet_t *pkt)
{
    uint8_t crc;

    crc = calc_crc8(0x00, &(pkt->all[PKT_HDR_IDX_TYPE]), PKT_HDR_SIZE-2+pkt->part.header.length);
    if(crc == pkt->part.header.crc8)
    {
        
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
            packet.all[packet_rxcnt++];
            if(packet_rxcnt == PKT_HDR_IDX_START)
            {
                if(rx_byte != HST_PACKET_START_BYTE) packet_rxcnt = 0;
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
            else if(packet_rxcnt >= PKT_HDR_IDX_LENGTH)
            {
                if(packet.part.header.type == TYPE_GET)
                {
                    // received full GET command and process it
                    on_recv_packet(&packet);
                    packet_rxcnt = 0;
                }
                else if(packet.part.header.type == TYPE_SET)
                {
                    if(packet.part.header.length <= 128)
                    {
                        if(packet_rxcnt == PKT_HDR_SIZE+packet.part.header.length)
                        {
                            // received full SET command and process it
                            on_recv_packet(&packet);
                            packet_rxcnt = 0;
                        }
                    }
                    else
                    {
                        packet_rxcnt = 0;
                    }
                }
                else
                {
                    packet_rxcnt = 0;
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
