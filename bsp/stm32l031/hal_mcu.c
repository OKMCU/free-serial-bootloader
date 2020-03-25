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
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_iwdg.h"
#include "hal_mcu.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void hal_mcu_init( void )
{
    LL_FLASH_EnablePreRead();
    LL_FLASH_EnablePrefetch();
     
    /* Enable Power Control clock */
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_PWR );
    
    /* The voltage scaling allows optimizing the power consumption when the device is 
       clocked below the maximum system frequency, to update the voltage scaling value 
       regarding system frequency refer to product datasheet.  */
    LL_PWR_SetRegulVoltageScaling( LL_PWR_REGU_VOLTAGE_SCALE1 );
    /* Disable Power Control clock */
    LL_APB1_GRP1_DisableClock( LL_APB1_GRP1_PERIPH_PWR );
    
    /* Enable HSI Oscillator */
    LL_RCC_HSI_Enable();
    while( !LL_RCC_HSI_IsReady() );
    
    LL_RCC_HSI_SetCalibTrimming( 16 );
    LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2 );
    LL_RCC_PLL_Enable();
    while( !LL_RCC_PLL_IsReady() );

    LL_RCC_LSI_Enable();
    while( !LL_RCC_LSI_IsReady() );

    LL_FLASH_SetLatency( LL_FLASH_LATENCY_1 );              // Range 1 (1.65V~1.95V), 32MHz, Flash latency should be set to 1
                                                            // Refer to RM0377, section 3.7.1.
    LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );     // Use PLL output as SYSCLK
    LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );          // HCLK 32MHz
    LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_1 );           // APB1 CLK 32MHz
    LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );           // APB2 CLK 32MHz

    LL_IOP_GRP1_EnableClock( LL_IOP_GRP1_PERIPH_GPIOA );
    LL_IOP_GRP1_EnableClock( LL_IOP_GRP1_PERIPH_GPIOB );
    LL_IOP_GRP1_EnableClock( LL_IOP_GRP1_PERIPH_GPIOC );

    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_8);
    LL_IWDG_SetReloadCounter(IWDG, 400);  // 1/32kHz*8*400, 100ms refresh timeout
   
    //NVIC_SetPriority( I2C1_IRQn,      HAL_NVIC_PRIORITY_I2C    ); // Priority 0 (Highest)
    //NVIC_SetPriority( ADC1_COMP_IRQn, HAL_NVIC_PRIORITY_ADC    ); // Priority 3 (Lowest)
    //NVIC_SetPriority( EXTI4_15_IRQn,  HAL_NVIC_PRIORITY_EXTINT ); // Priority 2
    //NVIC_SetPriority( EXTI0_1_IRQn,   HAL_NVIC_PRIORITY_EXTINT ); // Priority 2
    //NVIC_SetPriority( EXTI2_3_IRQn,   HAL_NVIC_PRIORITY_EXTINT ); // Priority 2

    //NVIC_EnableIRQ( I2C1_IRQn );
    //NVIC_EnableIRQ( ADC1_COMP_IRQn );
    //NVIC_EnableIRQ( EXTI4_15_IRQn );
    //NVIC_EnableIRQ( EXTI0_1_IRQn );
    //NVIC_EnableIRQ( EXTI2_3_IRQn );

}

void hal_mcu_wdg_start(void)
{
    LL_IWDG_Enable(IWDG);
    while(!LL_IWDG_IsReady(IWDG));
}

void hal_mcu_wdg_reset(void)
{
    LL_IWDG_ReloadCounter(IWDG);
}

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/
