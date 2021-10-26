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
#include "crc.h"
/* Private define ------------------------------------------------------------*/
/* Maximum number of bytes in payload of a packet */
#define PKT_PLD_SIZE                        128
/* Packet type enum */
#define TYPE_SET                            0x01
#define TYPE_GET                            0x02
#define TYPE_SUCCESS                        0x80
#define TYPE_FAILURE_UNKNOWN_REG            (0x80|0x00)
#define TYPE_FAILURE_ERR_LENGTH             (0x80|0x01)
#define TYPE_FAILURE_NOT_SUPPORT            (0x80|0x02)
#define TYPE_FAILURE_ERR_PASSWD             (0x80|0x03)
#define TYPE_FAILURE_ERR_SIGNATURE          (0x80|0x04)
#define TYPE_FAILURE_ERR_HAL                (0x80|0x05)
#define TYPE_FAILURE_ERR_PARAM              (0x80|0x06)
/* Private typedef -----------------------------------------------------------*/
/**
 * First 4-byte header format of a packet.
 */
typedef struct packet_header_s {
    /* device address, ranging from 0x01 ~ 0xFF, 0x00 is for broadcasting */
    uint8_t dev_addr;
    /* packet type */
    uint8_t type;
    /* register address */
    uint8_t reg_addr;
    /** 
     * number of bytes in payload of a SET type packet, or
     * number of bytes to read of a GET type packet
     */
    uint8_t length;
} packet_header_t;

/**
 * Full packet format.
 */
typedef struct packet_s {
    /* packet header */
    packet_header_t header;
    /* packet payload */
    uint8_t payload[PKT_PLD_SIZE];
} packet_t;

/**
 * register handlers
 */
typedef struct reg_handler_s {
    void (*p_fxn_set_reg)(uint8_t reg_addr, uint8_t *payload, uint8_t length);
    void (*p_fxn_get_reg)(uint8_t reg_addr, uint8_t length);
} reg_handler_t;

/**
 * FLASH/EEPROM memory read/write control body
 */
typedef struct nvm_ctrl_s {
    /* selected NVM page, <0 for not selected */
    int32_t page;
    /* starting address of the selected NVM page */
    uint32_t addr;
    /* number of bytes in the selected NVM page */
    uint32_t size;
    /* offset address of the selected NVM page */
    uint32_t offs;
} nvm_ctrl_t;
/* Private macro -------------------------------------------------------------*/
/**
 * break a 32-bit value into bytes
 */
#define BREAK_UINT32(var, ByteNum) \
          (uint8_t)((uint32_t)(((var) >>((ByteNum) * 8)) & 0x00FF))

/**
 * build a 32-bit value from bytes
 */
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

/**
 * build a 16-bit value from bytes
 */
#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

/**
 * break a 16-bit value into bytes
 */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

/**
 * bitmask value of one bit
 */
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
static void get_reg_24h(uint8_t reg_addr, uint8_t length);
static void get_reg_25h(uint8_t reg_addr, uint8_t length);
static void get_reg_29h(uint8_t reg_addr, uint8_t length);
static void get_reg_2Ah(uint8_t reg_addr, uint8_t length);
static void get_reg_2Bh(uint8_t reg_addr, uint8_t length);
static void get_reg_2Ch(uint8_t reg_addr, uint8_t length);
static void get_reg_2Dh(uint8_t reg_addr, uint8_t length);
/* Private variables ---------------------------------------------------------*/
/**
 * password OK flag
 * 1 == password check pass
 * 0 == password check fail
 */
static uint8_t passwd_ok = 0;
/* buffer to receive uart data */
static uint8_t packet[256];
/* FLASH memory read/write controller */
static nvm_ctrl_t fmc = {.page = -1};
/* EEPROM memory read/write controller */
static nvm_ctrl_t emc = {.page = -1};

#if defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
/* packet error counter */
static uint32_t pkt_err_cnt = 0;
/* selected baud rate */
static uint8_t baudrate_sel = 0;
#endif //defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)

/**
 * GET/SET handlers of a specified register address
 * if NULL, set_reg_XXh() or get_reg_XXh() would be invoked instead
 */
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
    {NULL,        NULL       }, //23h
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

#if defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
/* uart baud rate list */
static const uint32_t baudrate_lut[] = {
    2400,
    4800,
    9600,
    14400,
    19200,
    38400,
    56000,
    57600,
    115200,
    128000,
    230400,
    256000,
    460800,
    921600,
    1000000,
    2000000
};
#endif //defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  send a packet to host side
  * @param  type [I] - packet type
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes in payload
  * @param  payload [I] - points to the payload data
  * @retval 0 = success, -1 = failure
  */
static int8_t send_packet(uint8_t type, uint8_t reg_addr, uint8_t length, void *payload)
{
    uint8_t crc;
    packet_header_t pkt_header;
    if(length > PKT_PLD_SIZE) return -1;

    pkt_header.dev_addr = DEVICE_ADDR;
    pkt_header.type = type;
    pkt_header.reg_addr = reg_addr;
    pkt_header.length = length;

    crc = crc8_maxim((uint8_t *)&pkt_header, sizeof(pkt_header));
    if(length > 0)
        crc = crc8_maxim_update(crc, payload, length);

    hal_uart_send((uint8_t *)&pkt_header, sizeof(pkt_header));
    if(length > 0)
        hal_uart_send((uint8_t *)payload, length);
    hal_uart_send((uint8_t *)crc, 1);

    return 0;
}

/**
  * @brief  process the received packet
  * @param  pkt [I] - the recived packet
  * @retval 0 = success, -1 = failure
  */
static int8_t on_recv_packet(packet_t *pkt)
{
    if(pkt->header.dev_addr != DEVICE_ADDR) return -1;

    if(pkt->header.type == TYPE_SET)
    {
        if(pkt->header.reg_addr < sizeof(hdl)/sizeof(hdl[0]))
        {
            if(hdl[pkt->header.reg_addr].p_fxn_set_reg != NULL)
                hdl[pkt->header.reg_addr].p_fxn_set_reg(pkt->header.reg_addr, pkt->header.length > 0 ? pkt->payload : NULL, pkt->header.length);
            else
                set_reg_XXh(pkt->header.reg_addr, pkt->payload, pkt->header.length);
        }
        else
        {
            set_reg_XXh(pkt->header.reg_addr, pkt->payload, pkt->header.length);
        }
    }
    else if(pkt->header.type == TYPE_GET)
    {
        if(pkt->header.reg_addr < sizeof(hdl)/sizeof(hdl[0]))
        {
            if(hdl[pkt->header.reg_addr].p_fxn_get_reg != NULL)
                hdl[pkt->header.reg_addr].p_fxn_get_reg(pkt->header.reg_addr, pkt->header.length);
            else
                get_reg_XXh(pkt->header.reg_addr, pkt->header.length);
        }
        else
        {
            get_reg_XXh(pkt->header.reg_addr, pkt->header.length);
        }
    }
    else
    {
        return -1;
    }

    return 0;
}

/**
  * @brief  Default register SET handler. Directly returns error code to host.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_XXh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    send_packet(TYPE_FAILURE_NOT_SUPPORT, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x0E. Check password.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_0Eh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    if(length == strlen(BLDR_PASSWORD))
    {
        if(strncmp(BLDR_PASSWORD, (char *)payload, length) == 0)
        {
            passwd_ok = 1;
            send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
            return;
        }
    }
    passwd_ok = 0;
    send_packet(TYPE_FAILURE_ERR_PARAM, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x0F. Commit APROM image.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_0Fh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    uint32_t s;
    if(length != 4)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }
    s = BUILD_UINT32(payload[3], payload[2],payload[1], payload[0]);
    if(s != COMMIT_IMG_SIGNATURE)
    {
        send_packet(TYPE_FAILURE_ERR_SIGNATURE, reg_addr, 0, NULL);
        return;
    }
    if(hal_eeprom_write(EEPROM_ADDR_COMMIT_IMG_SIGNATURE, &s, sizeof(s)) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
    hal_reset();
}

/**
  * @brief  SET register 0x18. FLASH page erase or calculate page CRC.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_18h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    uint8_t buf[128];
    uint32_t page_addr;
    uint32_t page_size;
    uint32_t i = 0;
    uint32_t l;
    uint8_t crc = 0x00;
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }
    if(length != 1)
    {
        send_packet(TYPE_FAILURE_ERR_PARAM, reg_addr, 0, NULL);
        return;
    }

    switch(payload[0])
    {
        case 0x01:
            if(hal_flash_page_erase(fmc.page) != HAL_OK)
            {
                send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
                return;
            }
            send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
        return;

        case 0x02:
            if(hal_flash_page_info(fmc.page, &page_addr, &page_size) != HAL_OK)
            {
                send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
                return;
            }
            while(i < page_size)
            {
                l = (i+sizeof(buf) < page_size) ? sizeof(buf) : (page_size-i);
                if(hal_flash_read(page_addr+i, buf, l) != HAL_OK)
                {
                    send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
                    return;
                }
                crc = crc8_maxim_update(crc, buf, l);
                i += l;
            }

            send_packet(TYPE_SUCCESS, reg_addr, sizeof(crc), &crc);
        return;

        default:
            send_packet(TYPE_FAILURE_NOT_SUPPORT, reg_addr, 0, NULL);
        return;
    }
}

/**
  * @brief  SET register 0x19. FLASH page selection.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_19h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    int32_t page;
    uint32_t addr, size;
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(length != 4)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }

    page = (int32_t)BUILD_UINT32(payload[3], payload[2], payload[1], payload[0]);

    if(hal_flash_page_info(page, &addr, &size) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }

    fmc.page = page;
    fmc.addr = addr;
    fmc.size = size;
    fmc.offs = 0;
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x1C. Set FLASH page offset pointer.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_1Ch(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    uint32_t offs;
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(length != 4)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }

    offs = BUILD_UINT32(payload[3], payload[2], payload[1], payload[0]);
    if(offs >= fmc.size)
    {
        send_packet(TYPE_FAILURE_ERR_PARAM, reg_addr, 0, NULL);
        return;
    }
    fmc.offs = offs;
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x1D. Write data into FLASH.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_1Dh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(hal_flash_write(fmc.addr+fmc.offs, payload, length) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }

    fmc.offs += length;
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x28. EEPROM page erase or calculate page CRC.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_28h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    uint8_t buf[128];
    uint32_t page_addr;
    uint32_t page_size;
    uint32_t i = 0;
    uint32_t l;
    uint8_t crc;
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }
    if(length != 1)
    {
        send_packet(TYPE_FAILURE_ERR_PARAM, reg_addr, 0, NULL);
        return;
    }

    switch(payload[0])
    {
        case 0x01:
            if(hal_eeprom_page_erase(emc.page) != HAL_OK)
            {
                send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
                return;
            }
            send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
        return;

        case 0x02:
            if(hal_eeprom_page_info(emc.page, &page_addr, &page_size) != HAL_OK)
            {
                send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
                return;
            }
            while(i < page_size)
            {
                l = (i+sizeof(buf) < page_size) ? sizeof(buf) : (page_size-i);
                if(hal_eeprom_read(page_addr+i, buf, l) != HAL_OK)
                {
                    send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
                    return;
                }
                crc = crc8_maxim(buf, l);
                i += l;
            }

            send_packet(TYPE_SUCCESS, reg_addr, sizeof(crc), &crc);
        return;

        default:
            send_packet(TYPE_FAILURE_NOT_SUPPORT, reg_addr, 0, NULL);
        return;
    }
}

/**
  * @brief  SET register 0x29. EEPROM page selection.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_29h(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    int32_t page;
    uint32_t addr, size;
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(length != 4)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }

    page = (int32_t)BUILD_UINT32(payload[3], payload[2], payload[1], payload[0]);

    if(hal_eeprom_page_info(page, &addr, &size) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }

    emc.page = page;
    emc.addr = addr;
    emc.size = size;
    emc.offs = 0;
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x2C. Set EEPROM page offset pointer.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_2Ch(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    uint32_t offs;
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(length != 4)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }

    offs = BUILD_UINT32(payload[3], payload[2], payload[1], payload[0]);
    if(offs >= emc.size)
    {
        send_packet(TYPE_FAILURE_ERR_PARAM, reg_addr, 0, NULL);
        return;
    }
    emc.offs = offs;
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
}

/**
  * @brief  SET register 0x2D. Write data into EEPROM.
  * @param  reg_addr [I] - register address
  * @param  payload [I] - points to the payload data
  * @param  length [I] - number of bytes in payload data
  * @retval none
  */
static void set_reg_2Dh(uint8_t reg_addr, uint8_t *payload, uint8_t length)
{
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(hal_eeprom_write(emc.addr+emc.offs, payload, length) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }

    emc.offs += length;
    send_packet(TYPE_SUCCESS, reg_addr, 0, NULL);
}

/**
  * @brief  Default register GET handler. Directly returns error code to host.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_XXh(uint8_t reg_addr, uint8_t length)
{
    send_packet(TYPE_FAILURE_NOT_SUPPORT, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x00. Returns MCU part number.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_00h(uint8_t reg_addr, uint8_t length)
{
    char payload[PKT_PLD_SIZE];
    uint8_t l;
    memset(payload, 0, sizeof(payload));
    l = strlen(MCU_PART_NUMBER);
    memcpy(payload, MCU_PART_NUMBER, l);
    if(length >= l && length <= PKT_PLD_SIZE)
        send_packet(TYPE_SET, reg_addr, l, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x01. Returns MCU UUID.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_01h(uint8_t reg_addr, uint8_t length)
{
    char payload[PKT_PLD_SIZE];
    uint8_t l;
    memset(payload, 0, sizeof(payload));
    l = hal_mcu_get_uuid(payload, sizeof(payload));
    if(length >= l && length <= PKT_PLD_SIZE)
        send_packet(TYPE_SET, reg_addr, l, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x02. Returns bootloader version.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_02h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BLDR_PROG_MAJOR_VER, 
        BLDR_PROG_MINOR_VER, 
        HI_UINT16(BLDR_PROG_BUILD_VER), 
        LO_UINT16(BLDR_PROG_BUILD_VER)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x03. Returns bootloader start FLASH address.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_03h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 3),
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 2),
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 1),
        BREAK_UINT32(FLASH_ADDR_BLDR_START, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x04. Returns bootloader size.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_04h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_BLDR_SIZE, 3),
        BREAK_UINT32(FLASH_BLDR_SIZE, 2),
        BREAK_UINT32(FLASH_BLDR_SIZE, 1),
        BREAK_UINT32(FLASH_BLDR_SIZE, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x10. Returns FLASH start address.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_10h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_ADDR_START, 3),
        BREAK_UINT32(FLASH_ADDR_START, 2),
        BREAK_UINT32(FLASH_ADDR_START, 1),
        BREAK_UINT32(FLASH_ADDR_START, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x11. Returns FLASH total size.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_11h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_TOTAL_SIZE, 3),
        BREAK_UINT32(FLASH_TOTAL_SIZE, 2),
        BREAK_UINT32(FLASH_TOTAL_SIZE, 1),
        BREAK_UINT32(FLASH_TOTAL_SIZE, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x12. Returns number of FLASH sectors.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_12h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_PAGE_NUM, 3),
        BREAK_UINT32(FLASH_PAGE_NUM, 2),
        BREAK_UINT32(FLASH_PAGE_NUM, 1),
        BREAK_UINT32(FLASH_PAGE_NUM, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x13. Returns supported FLASH read/program width.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
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
        send_packet(TYPE_SET, reg_addr, length, &payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x14. Returns Max. FLASH page erase time in ms.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_14h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 3),
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 2),
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 1),
        BREAK_UINT32(FLASH_PAGE_ERASE_TIME, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x15. Returns Max. FLASH page program time in ms.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_15h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 3),
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 2),
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 1),
        BREAK_UINT32(FLASH_PAGE_PROG_TIME, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x19. Returns selected FLASH page ID.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_19h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(fmc.page, 3);
    payload[1] = BREAK_UINT32(fmc.page, 2);
    payload[2] = BREAK_UINT32(fmc.page, 1);
    payload[3] = BREAK_UINT32(fmc.page, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x1A. Returns FLASH page start address.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_1Ah(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(fmc.addr, 3);
    payload[1] = BREAK_UINT32(fmc.addr, 2);
    payload[2] = BREAK_UINT32(fmc.addr, 1);
    payload[3] = BREAK_UINT32(fmc.addr, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x1B. Returns FLASH page size.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_1Bh(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(fmc.size, 3);
    payload[1] = BREAK_UINT32(fmc.size, 2);
    payload[2] = BREAK_UINT32(fmc.size, 1);
    payload[3] = BREAK_UINT32(fmc.size, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x1C. Returns FLASH address offset pointer.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_1Ch(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(fmc.offs, 3);
    payload[1] = BREAK_UINT32(fmc.offs, 2);
    payload[2] = BREAK_UINT32(fmc.offs, 1);
    payload[3] = BREAK_UINT32(fmc.offs, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x1D. Returns data read from FLASH.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_1Dh(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[PKT_PLD_SIZE];
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(length == 0 || length > PKT_PLD_SIZE)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }

    if(hal_flash_read(fmc.addr+fmc.offs, payload, length) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }

    send_packet(TYPE_SET, reg_addr, length, payload);
}

/**
  * @brief  GET register 0x20. Returns (virtual) EEPROM's FLASH start address.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
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
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
#else
    send_packet(TYPE_FAILURE_NOT_SUPPORT, reg_addr, 0, NULL);
#endif
}

/**
  * @brief  GET register 0x21. Returns EEPROM total size.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_21h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(EEPROM_TOTAL_SIZE, 3),
        BREAK_UINT32(EEPROM_TOTAL_SIZE, 2),
        BREAK_UINT32(EEPROM_TOTAL_SIZE, 1),
        BREAK_UINT32(EEPROM_TOTAL_SIZE, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x22. Returns number of EEPROM sectors.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_22h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(EEPROM_PAGE_NUM, 3),
        BREAK_UINT32(EEPROM_PAGE_NUM, 2),
        BREAK_UINT32(EEPROM_PAGE_NUM, 1),
        BREAK_UINT32(EEPROM_PAGE_NUM, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x24. Returns Max. EEPROM page erase time in ms.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_24h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(EEPROM_PAGE_ERASE_TIME, 3),
        BREAK_UINT32(EEPROM_PAGE_ERASE_TIME, 2),
        BREAK_UINT32(EEPROM_PAGE_ERASE_TIME, 1),
        BREAK_UINT32(EEPROM_PAGE_ERASE_TIME, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x25. Returns Max. EEPROM page program time in ms.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_25h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4] = {
        BREAK_UINT32(EEPROM_PAGE_PROG_TIME, 3),
        BREAK_UINT32(EEPROM_PAGE_PROG_TIME, 2),
        BREAK_UINT32(EEPROM_PAGE_PROG_TIME, 1),
        BREAK_UINT32(EEPROM_PAGE_PROG_TIME, 0)
    };
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x29. Returns selected EEPROM page ID.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_29h(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(emc.page, 3);
    payload[1] = BREAK_UINT32(emc.page, 2);
    payload[2] = BREAK_UINT32(emc.page, 1);
    payload[3] = BREAK_UINT32(emc.page, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x2A. Returns EEPROM page start address.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_2Ah(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(emc.addr, 3);
    payload[1] = BREAK_UINT32(emc.addr, 2);
    payload[2] = BREAK_UINT32(emc.addr, 1);
    payload[3] = BREAK_UINT32(emc.addr, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x2B. Returns EEPROM page size.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_2Bh(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(emc.size, 3);
    payload[1] = BREAK_UINT32(emc.size, 2);
    payload[2] = BREAK_UINT32(emc.size, 1);
    payload[3] = BREAK_UINT32(emc.size, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x2C. Returns EEPROM address offset pointer.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_2Ch(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[4];
    payload[0] = BREAK_UINT32(emc.offs, 3);
    payload[1] = BREAK_UINT32(emc.offs, 2);
    payload[2] = BREAK_UINT32(emc.offs, 1);
    payload[3] = BREAK_UINT32(emc.offs, 0);
    if(length == 4)
        send_packet(TYPE_SET, reg_addr, length, payload);
    else
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
}

/**
  * @brief  GET register 0x2D. Returns data read from EEPROM.
  * @param  reg_addr [I] - register address
  * @param  length [I] - number of bytes requested by host
  * @retval none
  */
static void get_reg_2Dh(uint8_t reg_addr, uint8_t length)
{
    uint8_t payload[PKT_PLD_SIZE];
    if(!passwd_ok)
    {
        send_packet(TYPE_FAILURE_ERR_PASSWD, reg_addr, 0, NULL);
        return;
    }

    if(length == 0 || length > PKT_PLD_SIZE)
    {
        send_packet(TYPE_FAILURE_ERR_LENGTH, reg_addr, 0, NULL);
        return;
    }

    if(hal_eeprom_read(emc.addr+emc.offs, payload, length) != HAL_OK)
    {
        send_packet(TYPE_FAILURE_ERR_HAL, reg_addr, 0, NULL);
        return;
    }

    send_packet(TYPE_SET, reg_addr, length, payload);
}
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void bootloader_reset_handler(void)
{
    uint32_t signature;
    uint32_t appl;
    /* read bootloader signature to check if APPCODE has been commited */
    if(hal_eeprom_read(EEPROM_ADDR_COMMIT_IMG_SIGNATURE, &signature, sizeof(signature)) == HAL_OK)
    {
        if(signature == COMMIT_IMG_SIGNATURE)
        {
            /* read first 4 bytes at starting address of APPCODE */
            if(hal_flash_read(FLASH_ADDR_APPCODE_START, &appl, 4) == HAL_OK)
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
    uint32_t rxlen;
    uint32_t signature;

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    /* start watch dog */
    hal_wdg_config(HAL_WDG_TIMEOUT);
    hal_wdg_start();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)

    /**
     * If APROM is committed but somehow there no data at start address of APROM, 
     * erase the APROM committed signature to avoid running a half programmed image in APROM.
     */
    if(hal_eeprom_read(EEPROM_ADDR_COMMIT_IMG_SIGNATURE, &signature, sizeof(signature)) == HAL_OK)
    {
        if(signature == COMMIT_IMG_SIGNATURE)
        {
            signature = 0xFFFFFFFF;
            hal_eeprom_write(EEPROM_ADDR_COMMIT_IMG_SIGNATURE, &signature, sizeof(signature));
        }
    }

    /* set default UART baud rate */
    hal_uart_config(HAL_UART_BAUDRATE);

    while(1)
    {
#if defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
        pkt_err_cnt++;
#endif //defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
        /* pending here until receives a full packet or error happens during receiving */
        if(hal_uart_recv(packet, sizeof(packet), &rxlen) == HAL_OK)
        {
            /* check packet CRC */
            if(rxlen > sizeof(packet_header_t) && packet[rxlen-1] == crc8_maxim(packet, rxlen-1))
            {
                /* check CRC passed and process the packet */
                if(on_recv_packet((packet_t *)packet) == 0)
                {
#if defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
                    pkt_err_cnt = 0;
#endif //defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
                }
            }
        }

#if defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
        /* adjust the baud rate if error count exceeds the size of the packet header */
        if(pkt_err_cnt > sizeof(packet_header_t))
        {
            do
            {
                baudrate_sel++;
                if(baudrate_sel >= sizeof(baudrate_lut)/sizeof(baudrate_lut[0]))
                {
                    baudrate_sel = 0;
                }
            }
            while(hal_uart_config(baudrate_lut[baudrate_sel]) != HAL_OK);
            pkt_err_cnt = 0;
        }
#endif //defined (BAUDRATE_AUTO_DETECTION_EN) && (BAUDRATE_AUTO_DETECTION_EN > 0)
    }
}
/******************************** END OF FILE *********************************/
