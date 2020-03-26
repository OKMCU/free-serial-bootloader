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
#include "stm32l0xx.h"
#include "hal_flash.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FLASH_PEKEY1        0x89ABCDEF
#define FLASH_PEKEY2        0x02030405
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_flash_erase_page( uint32_t addr )
{
    /* (1) Wait till no operation is on going */
    /* (2) Check if the PELOCK is unlocked */
    /* (3) Perform unlock sequence */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
    {
        /* For robust implementation, add here time-out management */
    }
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* (2) */
    {
        FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */
        FLASH->PEKEYR = FLASH_PEKEY2;
    }


    /* (1) Set the ERASE and PROG bits in the FLASH_PECR register
           to enable page erasing */
    /* (2) Write a 32-bit word value in an address of the selected page
           to start the erase sequence */
    /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
    /* (4) Check the EOP flag in the FLASH_SR register */
    /* (5) Clear EOP flag by software by writing EOP at 1 */
    /* (6) Reset the ERASE and PROG bits in the FLASH_PECR register
           to disable the page erase */
    FLASH->PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG; /* (1) */
    *(__IO uint32_t *)addr = (uint32_t)0; /* (2) */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */
    {
        /* For robust implementation, add here time-out management */
    }
    if ((FLASH->SR & FLASH_SR_EOP) != 0) /* (4) */
    {
        FLASH->SR = FLASH_SR_EOP; /* (5) */
    }
    else
    {
        /* Manage the error cases */
    }
    FLASH->PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG); /* (6) */
    
    /* (1) Wait till no operation is on going */
    /* (2) Locks the NVM by setting PELOCK in PECR */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
    {
        /* For robust implementation, add here time-out management */
    }
    FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */
}

uint8_t hal_flash_read( uint32_t addr )
{
    return *((__IO uint8_t *)(addr));
}

void hal_flash_write( uint32_t addr, uint8_t *pdata, uint16_t len )
{
    uint16_t i;
    uint32_t data = 0;
    
    if((addr%HAL_FLASH_CELL_WIDTH) != 0 || (len%HAL_FLASH_CELL_WIDTH) != 0)
        return;
    
    /* (1) Wait till no operation is on going */
    /* (2) Check if the PELOCK is unlocked */
    /* (3) Perform unlock sequence */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
    {
        /* For robust implementation, add here time-out management */
    }
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) /* (2) */
    {
        FLASH->PEKEYR = FLASH_PEKEY1; /* (3) */
        FLASH->PEKEYR = FLASH_PEKEY2;
    }

    for(i = 0; i < len; i++, addr += HAL_FLASH_CELL_WIDTH)
    {
        data <<= 8;
        data |= pdata[i];
        if((i%HAL_FLASH_CELL_WIDTH) == 0)
        {
            /* (1) Perform the data write (32-bit word) at the desired address */
            /* (2) Wait until the BSY bit is reset in the FLASH_SR register */
            /* (3) Check the EOP flag in the FLASH_SR register */
            /* (4) clear it by software by writing it at 1 */
            *(__IO uint32_t*)(addr) = data; /* (1) */
            while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (2) */
            {
                /* For robust implementation, add here time-out management */
            }
            
            if ((FLASH->SR & FLASH_SR_EOP) != 0) /* (3) */
            {
                FLASH->SR = FLASH_SR_EOP; /* (4) */
            }
            else
            {
                /* Manage the error cases */
            }
        }
    }
    
    /* (1) Wait till no operation is on going */
    /* (2) Locks the NVM by setting PELOCK in PECR */
    while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */
    {
        /* For robust implementation, add here time-out management */
    }
    FLASH->PECR |= FLASH_PECR_PELOCK; /* (2) */
}

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/
