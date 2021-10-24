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
#include "gd32f4xx.h"
#include "hal.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct nvm_page_attr_s {
    uint32_t addr;
    uint32_t size;
} nvm_page_attr_t;

typedef struct uart_rx_ctrl_s {
    uint8_t *p_rxbuf;
    uint32_t size;
    uint32_t rxlen;
    uint8_t complete;
    uint8_t error;
} uart_rx_ctrl_t;
/* Private macro -------------------------------------------------------------*/
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))
/* Private variables ---------------------------------------------------------*/
static const nvm_page_attr_t flash_page_attr[FLASH_PAGE_NUM] = {
    {.addr = 0x08000000, .size = 1024ul*16ul }, // page 0
    {.addr = 0x08004000, .size = 1024ul*16ul }, // page 1
    {.addr = 0x08008000, .size = 1024ul*16ul }, // page 2
    {.addr = 0x0800C000, .size = 1024ul*16ul }, // page 3
    {.addr = 0x08010000, .size = 1024ul*64ul }, // page 4
    {.addr = 0x08020000, .size = 1024ul*128ul}, // page 5
    {.addr = 0x08040000, .size = 1024ul*128ul}, // page 6
    {.addr = 0x08060000, .size = 1024ul*128ul}, // page 7
    {.addr = 0x08080000, .size = 1024ul*128ul}, // page 8
    {.addr = 0x080A0000, .size = 1024ul*128ul}, // page 9
    {.addr = 0x080C0000, .size = 1024ul*128ul}, // page 10
    {.addr = 0x080E0000, .size = 1024ul*128ul}, // page 11
    {.addr = 0x08100000, .size = 1024ul*16ul }, // page 12
    {.addr = 0x08104000, .size = 1024ul*16ul }, // page 13
    {.addr = 0x08108000, .size = 1024ul*16ul }, // page 14
    {.addr = 0x0810C000, .size = 1024ul*16ul }, // page 15
    {.addr = 0x08110000, .size = 1024ul*64ul }, // page 16
    {.addr = 0x08120000, .size = 1024ul*128ul}, // page 17
    {.addr = 0x08140000, .size = 1024ul*128ul}, // page 18
    {.addr = 0x08160000, .size = 1024ul*128ul}, // page 19
    {.addr = 0x08180000, .size = 1024ul*128ul}, // page 20
    {.addr = 0x081A0000, .size = 1024ul*128ul}, // page 21
    {.addr = 0x081C0000, .size = 1024ul*128ul}, // page 22
    {.addr = 0x081E0000, .size = 1024ul*128ul}, // page 23
    {.addr = 0x08200000, .size = 1024ul*256ul}, // page 24
    {.addr = 0x08240000, .size = 1024ul*256ul}, // page 25
    {.addr = 0x08280000, .size = 1024ul*256ul}, // page 26
    {.addr = 0x082C0000, .size = 1024ul*256ul}  // page 27
};

static const nvm_page_attr_t eeprom_page_attr[EEPROM_PAGE_NUM] = {
    {.addr = 0x00000000, .size = 1024ul*16ul } // page 0
};

static const uint32_t CTL_SECTOR_NUMBER[FLASH_PAGE_NUM] = {
    CTL_SECTOR_NUMBER_0,
    CTL_SECTOR_NUMBER_1,
    CTL_SECTOR_NUMBER_2,
    CTL_SECTOR_NUMBER_3,
    CTL_SECTOR_NUMBER_4,
    CTL_SECTOR_NUMBER_5,
    CTL_SECTOR_NUMBER_6,
    CTL_SECTOR_NUMBER_7,
    CTL_SECTOR_NUMBER_8,
    CTL_SECTOR_NUMBER_9,
    CTL_SECTOR_NUMBER_10,
    CTL_SECTOR_NUMBER_11,
    CTL_SECTOR_NUMBER_12,
    CTL_SECTOR_NUMBER_13,
    CTL_SECTOR_NUMBER_14,
    CTL_SECTOR_NUMBER_15,
    CTL_SECTOR_NUMBER_16,
    CTL_SECTOR_NUMBER_17,
    CTL_SECTOR_NUMBER_18,
    CTL_SECTOR_NUMBER_19,
    CTL_SECTOR_NUMBER_20,
    CTL_SECTOR_NUMBER_21,
    CTL_SECTOR_NUMBER_22,
    CTL_SECTOR_NUMBER_23,
    CTL_SECTOR_NUMBER_24,
    CTL_SECTOR_NUMBER_25,
    CTL_SECTOR_NUMBER_26,
    CTL_SECTOR_NUMBER_27
};

static uint8_t eepbuf[EEPROM_TOTAL_SIZE];
static uint8_t eepbuf_init = 0;
static volatile uart_rx_ctrl_t uart_rx_ctrl;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_enter_critical(void)
{
    __disable_irq();
}

void hal_exit_critical(void)
{
    __enable_irq();
}

void hal_reset(void)
{
    NVIC_SystemReset();
}

void hal_jump_to_addr(uint32_t addr)
{
    /* set up stack pointer and jump to application reset vector */  
    asm volatile("LDR    r1, [%0]    \n"
                 "MOV    sp, r1      \n"
                 "ADDS   r0,r0,#0x04 \n"
                 "LDR    r0, [r0]    \n"
                 "BX     r0          \n"
                 :
                 : "r"(addr)
                 : "cc");
}

uint8_t hal_mcu_get_uuid(char *p_uuid, uint8_t size)
{
    strncpy(p_uuid, "MCU_UUID", size);
    return (uint8_t)strlen("MCU_UUID");
}

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
int8_t hal_wdg_config(uint16_t timeout)
{
    uint32_t counter = timeout * 32;
    uint8_t div = 2;
    while((counter>>div) > 0x1000)
    {
        div++;
        if(div >= 8) break;
    }

    counter >>= div;
    if(counter > 0x1000) counter = 0x1000;
    counter--;

    fwdgt_config((uint16_t)counter, ((uint8_t)PSC_PSC(div-2)));
    return HAL_OK;
}

int8_t hal_wdg_start(void)
{
    uint32_t counter = timeout * 32;
    uint8_t div = 2;
    while((counter>>div) > 0x1000)
    {
        div++;
        if(div >= 8) break;
    }

    counter >>= div;
    if(counter > 0x1000) counter = 0x1000;
    counter--;

    fwdgt_config((uint16_t)counter, ((uint8_t)PSC_PSC(div-2)));
    fwdgt_enable();
    return HAL_OK;
}

int8_t hal_wdg_refresh(void)
{
    fwdgt_counter_reload();
    return HAL_OK;
}
#endif //defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)

int8_t hal_flash_page_info(int32_t page, uint32_t *addr, uint32_t *size)
{
    if(page < 0 || page >= FLASH_PAGE_NUM)
        return HAL_ERR;

    *addr = flash_page_attr[page].addr;
    *size = flash_page_attr[page].size;
    return HAL_OK;
}

int8_t hal_flash_page_erase(int32_t page)
{
    fmc_state_enum status;

    /* prevent user to erase pages where stores bootloader program */
    if(page <= 0 || page >= FLASH_PAGE_NUM)
        return HAL_ERR;

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    hal_wdg_config(FLASH_PAGE_ERASE_TIME);
    hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    /* unlock the flash program erase controller */
    fmc_unlock();
    /* clear pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    /* wait the erase operation complete*/
    status = fmc_sector_erase(CTL_SECTOR_NUMBER[page]);
    /* lock the flash program erase controller */
    fmc_lock();

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    hal_wdg_config(HAL_WDG_TIMEOUT);
    hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)

    return status == FMC_READY ? HAL_OK : HAL_ERR;
}

int8_t hal_flash_read(uint32_t addr, void *p_buf, uint32_t size)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)p_buf;

    if((addr < FLASH_ADDR_START) || ((addr+size) > (FLASH_ADDR_END+1)))
        return HAL_ERR;

    for(i = 0; i < size; i++)
    {
        p[i] = *((__IO uint8_t *)(addr++));
    }
    return HAL_OK;
}

int8_t hal_flash_write(uint32_t addr, const void *p_buf, uint32_t size)
{
    uint32_t tmp32, i = 0;
    uint16_t tmp16;
    const uint8_t *p = (const uint8_t *)p_buf;
    /* prevent user to program flash section where stores bootloader program */
    if((addr <= FLASH_ADDR_BLDR_END) || ((addr+size) > (FLASH_ADDR_END+1)))
        return HAL_ERR;

    /* unlock the flash program erase controller */
    fmc_unlock();

    if((addr%4) == 0 && (size%4) == 0)
    {
        // word (32-bit) program
        for(i = 0; i < size; i += 4)
        {
            tmp32 = BUILD_UINT32(p[i+3], p[i+2], p[i+1], p[i+0]);
            /* clear pending flags */
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
            /* program flash */
            if(fmc_word_program(addr+i, tmp32) != FMC_READY) break;
            /* check the programmed data */
            if(tmp32 != *((__IO uint32_t *)(addr+i))) break;
        }
    }
    else if((addr%2) == 0 && (size%2) == 0)
    {
        // half-word (16-bit) program
        for(i = 0; i < size; i += 2)
        {
            tmp16 = BUILD_UINT16(p[i+1], p[i+0]);
            /* clear pending flags */
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
            /* program flash */
            if(fmc_halfword_program(addr+i, tmp16) != FMC_READY) break;
            /* check the programmed data */
            if(tmp16 != *((__IO uint16_t *)(addr+i))) break;
        }
    }
    else
    {
        // byte (8-bit) program
        for(i = 0; i < size; i++)
        {
            /* clear pending flags */
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
            /* program flash */
            if(fmc_byte_program(addr+i, p[i]) != FMC_READY) break;
            /* check the programmed data */
            if(p[i] != *((__IO uint8_t *)(addr+i))) break;
        }
    }

    /* lock the flash program erase controller */
    fmc_lock();

    return (i == size) ? HAL_OK : HAL_ERR;
}

int8_t hal_eeprom_page_info(int32_t page, uint32_t *addr, uint32_t *size)
{
    if(page < 0 || page >= EEPROM_PAGE_NUM)
        return HAL_ERR;

    *addr = eeprom_page_attr[page].addr;
    *size = eeprom_page_attr[page].size;
    return HAL_OK;
}

int8_t hal_eeprom_page_erase(int32_t page)
{
    fmc_state_enum status;

    /* prevent user to erase pages where stores bootloader program */
    if(page <= 0 || page >= EEPROM_PAGE_NUM)
        return HAL_ERR;

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    hal_wdg_config(FLASH_PAGE_ERASE_TIME);
    hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    /* unlock the flash program erase controller */
    fmc_unlock();
    /* clear pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    /* wait the erase operation complete*/
    status = fmc_sector_erase(CTL_SECTOR_NUMBER_1);
    /* lock the flash program erase controller */
    fmc_lock();

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    hal_wdg_config(HAL_WDG_TIMEOUT);
    hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)

    eepbuf_init = 1;
    if(status != FMC_READY)
        return HAL_ERR;

    memset(eepbuf, 0xFF, sizeof(eepbuf));
    return HAL_OK;
}

int8_t hal_eeprom_read(uint32_t addr, void *p_buf, uint32_t size)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)p_buf;

    if((addr+size) > EEPROM_TOTAL_SIZE)
        return HAL_ERR;

    for(i = 0; i < size; i++)
    {
        p[i] = *((__IO uint8_t *)(FLASH_ADDR_VIRTUAL_EEPROM_START+(addr++)));
    }
    return HAL_OK;
}

int8_t hal_eeprom_write(uint32_t addr, const void *p_buf, uint32_t size)
{
    uint32_t i, tmp32;
    const uint8_t *p = (const uint8_t *)p_buf;
    fmc_state_enum status;

    if((addr+size) > EEPROM_TOTAL_SIZE)
        return HAL_ERR;

    if(!eepbuf_init)
    {
        eepbuf_init = 1;
        hal_flash_read(FLASH_ADDR_VIRTUAL_EEPROM_START, eepbuf, EEPROM_TOTAL_SIZE);
    }

    for(i = 0; i < size; i++)
    {
        eepbuf[addr+i] = p[i];
    }

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    hal_wdg_config(FLASH_PAGE_ERASE_TIME);
    hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    /* unlock the flash program erase controller */
    fmc_unlock();
    /* clear pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    /* wait the erase operation complete*/
    status = fmc_sector_erase(CTL_SECTOR_NUMBER_1);

#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    hal_wdg_config(HAL_WDG_TIMEOUT);
    hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)

    for(i = 0; i < EEPROM_TOTAL_SIZE; i += 4)
    {
#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
        hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
        tmp32 = BUILD_UINT32(eepbuf[i+3], eepbuf[i+2], eepbuf[i+1], eepbuf[i+0]);
        /* clear pending flags */
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
        /* program flash */
        if(fmc_word_program(FLASH_ADDR_VIRTUAL_EEPROM_START+i, tmp32) != FMC_READY) break;
        /* check the programmed data */
        if(tmp32 != *((__IO uint32_t *)(FLASH_ADDR_VIRTUAL_EEPROM_START+i))) break;
    }

    /* lock the flash program erase controller */
    fmc_lock();

    if((status != FMC_READY) || (i != EEPROM_TOTAL_SIZE))
        return HAL_ERR;

    return HAL_OK;
}

int8_t hal_uart_init(uint32_t baudrate)
{
    /* USART configure */
    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_baudrate_set(USART0, baudrate);
    //usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    usart_interrupt_enable(USART0, USART_INT_IDLE);
    usart_enable(USART0);
    return HAL_OK;
}

int8_t hal_uart_config(uint32_t baudrate)
{
    usart_baudrate_set(USART0, HAL_UART_BAUDRATE);
    return HAL_OK;
}

int8_t hal_uart_send(const uint8_t *p_data, uint32_t txlen)
{
    uint32_t i;
    for(i = 0; i < txlen; i++)
    {
        usart_data_transmit(USART0, p_data[i]);
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    }
    return HAL_OK;
}

int8_t hal_uart_recv(uint8_t *p_data, uint32_t size, uint32_t *rxlen)
{
    if(p_data == NULL || size == 0) return HAL_ERR;
    uart_rx_ctrl.p_rxbuf = p_data;
    uart_rx_ctrl.size = size;
    uart_rx_ctrl.rxlen = 0;
    uart_rx_ctrl.complete = 0;
    uart_rx_ctrl.error = 0;
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    while(uart_rx_ctrl.complete == 0 && uart_rx_ctrl.error == 0)
    {
#if defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
        /* refresh watch dog counter */
        hal_wdg_refresh();
#endif // defined (HAL_WDG_ENABLE) && (HAL_WDG_ENABLE > 0)
    }

    if(uart_rx_ctrl.complete)
    {
        *rxlen = uart_rx_ctrl.rxlen;
        return HAL_OK;
    }

    return HAL_ERR;
}

void hal_uart_isr(void)
{
    uint8_t byte;
    uint32_t rxlen;
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
    {
        /* receive data */
        byte = usart_data_receive(USART0);
        rxlen = uart_rx_ctrl.rxlen;
        if(rxlen < uart_rx_ctrl.size)
        {
            uart_rx_ctrl.p_rxbuf[rxlen++] = byte;
            if(rxlen == uart_rx_ctrl.size)
            {
                usart_receive_config(USART0, USART_RECEIVE_DISABLE);
                uart_rx_ctrl.complete = 1;
            }
            uart_rx_ctrl.rxlen = rxlen;
        }
    }

    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
    {
        /* clear interrupt flag */
        //usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
        byte = usart_data_receive(USART0);
        usart_receive_config(USART0, USART_RECEIVE_DISABLE);
        uart_rx_ctrl.complete = 1;
    }
}
/******************************** END OF FILE *********************************/
