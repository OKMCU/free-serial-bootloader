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
#define UART_BUF_SIZE                       1024
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
#define STS_FAILURE_WRONG_LENGTH            (0x80|0x01)

/* Private typedef -----------------------------------------------------------*/
typedef struct uart_rx_buffer_s {
    uint8_t data[UART_BUF_SIZE];
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

typedef struct reg_handler_s {
    void (*p_fxn_set_reg)(uint8_t reg_addr, uint8_t *payload, uint8_t length);
    void (*p_fxn_get_reg)(uint8_t reg_addr, uint8_t length);
} reg_handler_t;

typedef struct nvm_ctrl_s {
    int32_t sector_sel;
    uint32_t start_addr;
    uint32_t size;
    uint32_t offset;
} nvm_ctrl_t;
/* Private macro -------------------------------------------------------------*/
#define BREAK_UINT32(var, ByteNum) \
          (uint8_t)((uint32_t)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BIT(n)      (1<<n)
/* Private function prototypes -----------------------------------------------*/
static void set_reg_XXh(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_0Eh(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_0Fh(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_18h(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_19h(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_1Ch(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_1Dh(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_28h(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_29h(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_2Ch(uint8_t reg_addr, uint8_t *payload, uint8_t length);
static void set_reg_2Dh(uint8_t reg_addr, uint8_t *payload, uint8_t length);

static void get_reg_XXh(uint8_t reg_addr, uint8_t length);
static void get_reg_00h(uint8_t reg_addr, uint8_t length);
static void get_reg_01h(uint8_t reg_addr, uint8_t length);
static void get_reg_02h(uint8_t reg_addr, uint8_t length);
static void get_reg_03h(uint8_t reg_addr, uint8_t length);
static void get_reg_04h(uint8_t reg_addr, uint8_t length);
static void get_reg_10h(uint8_t reg_addr, uint8_t length);
static void get_reg_11h(uint8_t reg_addr, uint8_t length);
static void get_reg_12h(uint8_t reg_addr, uint8_t length);
static void get_reg_13h(uint8_t reg_addr, uint8_t length);
static void get_reg_14h(uint8_t reg_addr, uint8_t length);
static void get_reg_15h(uint8_t reg_addr, uint8_t length);
static void get_reg_19h(uint8_t reg_addr, uint8_t length);
static void get_reg_1Ah(uint8_t reg_addr, uint8_t length);
static void get_reg_1Bh(uint8_t reg_addr, uint8_t length);
static void get_reg_1Ch(uint8_t reg_addr, uint8_t length);
static void get_reg_1Dh(uint8_t reg_addr, uint8_t length);
static void get_reg_20h(uint8_t reg_addr, uint8_t length);
static void get_reg_21h(uint8_t reg_addr, uint8_t length);
static void get_reg_22h(uint8_t reg_addr, uint8_t length);
static void get_reg_23h(uint8_t reg_addr, uint8_t length);
static void get_reg_24h(uint8_t reg_addr, uint8_t length);
static void get_reg_25h(uint8_t reg_addr, uint8_t length);
static void get_reg_29h(uint8_t reg_addr, uint8_t length);
static void get_reg_2Ah(uint8_t reg_addr, uint8_t length);
static void get_reg_2Bh(uint8_t reg_addr, uint8_t length);
static void get_reg_2Ch(uint8_t reg_addr, uint8_t length);
static void get_reg_2Dh(uint8_t reg_addr, uint8_t length);
/* Private variables ---------------------------------------------------------*/
static volatile uart_rx_buffer_t rb;
static uint8_t packet_rxcnt = 0;
static uint8_t handshaking_cnt = 0;
static packet_t packet;
static nvm_ctrl_t flash_ctrl = {.sector_sel = -1};
static nvm_ctrl_t eeprom_ctrl = {.sector_sel = -1};
static const reg_handler_t hdl[] = {
    {NULL,        get_reg_00h}, //00h
    {NULL,        get_reg_01h}, //01h
    {NULL,        get_reg_02h}, //02h
    {NULL,        get_reg_03h}, //03h
    {NULL,        get_reg_04h}, //04h
    {NULL,        NULL       }, //05h
    {NULL,        NULL       }, //06h
    {NULL,        NULL       }, //07h
    {NULL,        NULL       }, //08h
    {NULL,        NULL       }, //09h
    {NULL,        NULL       }, //0Ah
    {NULL,        NULL       }, //0Bh
    {NULL,        NULL       }, //0Ch
    {NULL,        NULL       }, //0Dh
    {set_reg_0Eh, NULL       }, //0Eh
    {set_reg_0Fh, NULL       }, //0Fh
    {NULL,        get_reg_10h}, //10h
    {NULL,        get_reg_11h}, //11h
    {NULL,        get_reg_12h}, //12h
    {NULL,        get_reg_13h}, //13h
    {NULL,        get_reg_14h}, //14h
    {NULL,        get_reg_15h}, //15h
    {NULL,        NULL       }, //16h
    {NULL,        NULL       }, //17h
    {set_reg_18h, NULL       }, //18h
    {set_reg_19h, get_reg_19h}, //19h
    {NULL,        get_reg_1Ah}, //1Ah
    {NULL,        get_reg_1Bh}, //1Bh
    {set_reg_1Ch, get_reg_1Ch}, //1Ch
    {set_reg_1Dh, get_reg_1Dh}, //1Dh
    {NULL,        NULL       }, //1Eh
    {NULL,        NULL       }, //1Fh
    {NULL,        get_reg_20h}, //20h
    {NULL,        get_reg_21h}, //21h
    {NULL,        get_reg_22h}, //22h
    {NULL,        get_reg_23h}, //23h
    {NULL,        get_reg_24h}, //24h
    {NULL,        get_reg_25h}, //25h
    {NULL,        NULL       }, //26h
    {NULL,        NULL       }, //27h
    {set_reg_28h, NULL       }, //28h
    {set_reg_29h, get_reg_29h}, //29h
    {NULL,        get_reg_2Ah}, //2Ah
    {NULL,        get_reg_2Bh}, //2Bh
    {set_reg_2Ch, get_reg_2Ch}, //2Ch
    {set_reg_2Dh, get_reg_2Dh}, //2Dh
    {NULL,        NULL       }, //2Eh
    {NULL,        NULL       }, //2Fh
};
/* Private functions ---------------------------------------------------------*/
static void on_uart_recv_byte(uint8_t rx_byte)
{
    uint16_t curr_head, curr_tail, next_tail;

    curr_head = rb.head;
    curr_tail = rb.tail;
    next_tail = curr_tail+1;
    if(next_tail >= UART_BUF_SIZE) next_tail = 0;
    if(next_tail != curr_head)
    {
        rb.data[curr_tail] = rx_byte;
        rb.tail = next_tail;
    }
}

static uint8_t calc_crc8(uint8_t crc, void *p_buf, uint32_t size)
{
    return 0x00;
}

static int8_t send_packet(uint8_t type, uint8_t reg_addr, uint8_t status, uint8_t length, void *payload)
{
    uint8_t i, crc = 0x00;
    if(length > 128) return -1;
    if(type != TYPE_SET && type!= TYPE_GET) return -1;

    crc = calc_crc8(crc, &type, 1);
    crc = calc_crc8(crc, &reg_addr, 1);
    crc = calc_crc8(crc, &status, 1);
    crc = calc_crc8(crc, &length, 1);
    if(length > 0)
        crc = calc_crc8(crc, payload, length);
    hal_uart_send(DEV_PACKET_START_BYTE);
    hal_uart_send(crc);
    hal_uart_send(type);
    hal_uart_send(reg_addr);
    hal_uart_send(status);
    hal_uart_send(length);
    for(i = 0; i < length; i++)
    {
        hal_uart_send(((uint8_t *)payload)[i]);
    }

    return 0;
}

static void on_recv_packet(const packet_t *pkt)
{
    uint8_t crc;

    crc = calc_crc8(0x00, &(pkt->all[PKT_HDR_IDX_TYPE]), PKT_HDR_SIZE-2+pkt->part.header.length);
    if(crc == pkt->part.header.crc8)
    {
        if(pkt->part.header.type == TYPE_SET)
        {
            if(pkt->part.header.reg_addr < sizeof(hdl)/sizeof(hdl[0]))
            {
                if(hdl[pkt->part.header.reg_addr].p_fxn_set_reg != NULL)
                    hdl[pkt->part.header.reg_addr].p_fxn_set_reg(pkt->part.header.reg_addr, pkt->part.payload, pkt->part.header.length);
                else
                    set_reg_XXh(pkt->part.header.reg_addr, pkt->part.payload, pkt->part.header.length);
            }
            else
            {
                set_reg_XXh(pkt->part.header.reg_addr, pkt->part.payload, pkt->part.header.length);
            }
        }
        else if(pkt->part.header.type == TYPE_GET)
        {
            if(pkt->part.header.reg_addr < sizeof(hdl)/sizeof(hdl[0]))
            {
                if(hdl[pkt->part.header.reg_addr].p_fxn_get_reg != NULL)
                    hdl[pkt->part.header.reg_addr].p_fxn_get_reg(pkt->part.header.reg_addr, pkt->part.header.length);
                else
                    get_reg_XXh(pkt->part.header.reg_addr, pkt->part.header.length);
            }
            else
            {
                get_reg_XXh(pkt->part.header.reg_addr, pkt->part.header.length);
            }
        }
    }
}

static void set_reg_XXh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    send_packet(TYPE_SET, reg_addr, STS_FAILURE_UNKNOWN_REG, 0, NULL);
}
static void set_reg_0Eh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_0Fh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_18h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_19h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_1Ch(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_1Dh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_28h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_29h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_2Ch(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}
static void set_reg_2Dh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
}

static void get_reg_XXh(uint8_t reg_addr, uint8_t length)
{
    send_packet(TYPE_SET, reg_addr, STS_FAILURE_UNKNOWN_REG, 0, NULL);
}
static void get_reg_00h(uint8_t reg_addr, uint8_t length)
{
    char payload[128];
    uint8_t l;
    memset(payload, 0, sizeof(payload));
    l = strlen(MCU_PART_NUMBER);
    memcpy(payload, MCU_PART_NUMBER, l);
    if(length >= l && length <= 128)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, l, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_01h(uint8_t reg_addr, uint8_t length)
{
    char payload[128];
    uint8_t l;
    memset(payload, 0, sizeof(payload));
    l = hal_mcu_get_uuid(payload, sizeof(payload));
    if(length >= l && length <= 128)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, l, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_02h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BLDR_PROG_MAJOR_VER, 
        BLDR_PROG_MINOR_VER, 
        HI_UINT16(BLDR_PROG_BUILD_VER), 
        LO_UINT16(BLDR_PROG_BUILD_VER)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_03h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 3),
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 2),
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 1),
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_04h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_BLDR_SIZE, 3),
        BREAK_UINT32(FLASH_BLDR_SIZE, 2),
        BREAK_UINT32(FLASH_BLDR_SIZE, 1),
        BREAK_UINT32(FLASH_BLDR_SIZE, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_10h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_ADDR_START, 3),
        BREAK_UINT32(FLASH_ADDR_START, 2),
        BREAK_UINT32(FLASH_ADDR_START, 1),
        BREAK_UINT32(FLASH_ADDR_START, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_11h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_TOTAL_SIZE, 3),
        BREAK_UINT32(FLASH_TOTAL_SIZE, 2),
        BREAK_UINT32(FLASH_TOTAL_SIZE, 1),
        BREAK_UINT32(FLASH_TOTAL_SIZE, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_12h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_PAGE_NUM, 3),
        BREAK_UINT32(FLASH_PAGE_NUM, 2),
        BREAK_UINT32(FLASH_PAGE_NUM, 1),
        BREAK_UINT32(FLASH_PAGE_NUM, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_13h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload = (FLASH_PROG_QWORD<<7)|\
                      (FLASH_PROG_DWORD<<6)|\
                      (FLASH_PROG_WORD <<5)|\
                      (FLASH_PROG_BYTE <<4)|\
                      (FLASH_READ_QWORD<<3)|\
                      (FLASH_READ_DWORD<<2)|\
                      (FLASH_READ_WORD <<1)|\
                      (FLASH_READ_BYTE <<0);
    if(length == 1)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, &payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_14h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 3),
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 2),
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 1),
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_15h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 3),
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 2),
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 1),
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_19h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(flash_ctrl.sector_sel, 3);
    payload[1] = BREAK_UINT32(flash_ctrl.sector_sel, 2);
    payload[2] = BREAK_UINT32(flash_ctrl.sector_sel, 1);
    payload[3] = BREAK_UINT32(flash_ctrl.sector_sel, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_1Ah(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(flash_ctrl.start_addr, 3);
    payload[1] = BREAK_UINT32(flash_ctrl.start_addr, 2);
    payload[2] = BREAK_UINT32(flash_ctrl.start_addr, 1);
    payload[3] = BREAK_UINT32(flash_ctrl.start_addr, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_1Bh(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(flash_ctrl.size, 3);
    payload[1] = BREAK_UINT32(flash_ctrl.size, 2);
    payload[2] = BREAK_UINT32(flash_ctrl.size, 1);
    payload[3] = BREAK_UINT32(flash_ctrl.size, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_1Ch(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(flash_ctrl.offset, 3);
    payload[1] = BREAK_UINT32(flash_ctrl.offset, 2);
    payload[2] = BREAK_UINT32(flash_ctrl.offset, 1);
    payload[3] = BREAK_UINT32(flash_ctrl.offset, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
}
static void get_reg_1Dh(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_20h(uint8_t reg_addr, uint8_t length)
{
#if defined (FLASH_VIRTUAL_EEPROM_SIZE) && (FLASH_VIRTUAL_EEPROM_SIZE > 0)
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_ADDR_VIRTUAL_EEPROM_START, 3),
        BREAK_UINT32(FLASH_ADDR_VIRTUAL_EEPROM_START, 2),
        BREAK_UINT32(FLASH_ADDR_VIRTUAL_EEPROM_START, 1),
        BREAK_UINT32(FLASH_ADDR_VIRTUAL_EEPROM_START, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, STS_SUCCESS, length, payload);
    else
        send_packet(TYPE_SET, reg_addr, STS_FAILURE_WRONG_LENGTH, 0, NULL);
#else
    send_packet(TYPE_SET, reg_addr, STS_FAILURE_UNKNOWN_REG, 0, NULL);
#endif
}
static void get_reg_21h(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_22h(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_23h(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_24h(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_25h(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_29h(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_2Ah(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_2Bh(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_2Ch(uint8_t reg_addr, uint8_t length)
{
}
static void get_reg_2Dh(uint8_t reg_addr, uint8_t length)
{
}
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void bootloader_reset_handler(void)
{
    uint32_t signature;
    uint32_t appl;
    /* read bootloader signature to check if APPCODE has been commited */
    if(hal_eep_read(EEPROM_ADDR_COMMIT_IMG_SIGNATURE, &signature, sizeof(signature)) == HAL_OK)
    {
        if(signature == COMMIT_IMG_SIGNATUR)
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
            if(head >= UART_BUF_SIZE) head = 0;
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
