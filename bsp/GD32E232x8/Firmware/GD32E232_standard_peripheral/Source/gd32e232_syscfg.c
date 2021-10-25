/*!
    \file    gd32e232_syscfg.h
    \brief   SYSCFG driver

    \version 2019-05-15, V1.0.0, firmware for GD32E232
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e232_syscfg.h"

/*!
    \brief      reset the SYSCFG registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void syscfg_deinit(void)
{
    rcu_periph_reset_enable(RCU_CFGCMPRST);
    rcu_periph_reset_disable(RCU_CFGCMPRST);
}

/*!
    \brief      configure the boot mode 
    \param[in]  syscfg_bootmode: selects the boot mode
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_BOOTMODE_FLASH: boot from the main flash
      \arg        SYSCFG_BOOTMODE_SYSTEM: boot from the system flash memory
      \arg        SYSCFG_BOOTMODE_SRAM: boot from the embedded SRAM
    \param[out] none
    \retval     none
*/
void syscfg_bootmode_config(uint8_t syscfg_bootmode)
{
    /* reset the SYSCFG_CFG0_BOOT_MODE bit and set according to syscfg_bootmode */
    SYSCFG_CFG0 &= ~SYSCFG_CFG0_BOOT_MODE;
    SYSCFG_CFG0 |= (uint32_t)syscfg_bootmode;
}

/*!
    \brief      enable the DMA channels remapping
    \param[in]  syscfg_dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_DMA_REMAP_TIMER16: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        SYSCFG_DMA_REMAP_TIMER15: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        SYSCFG_DMA_REMAP_USART0RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        SYSCFG_DMA_REMAP_USART0TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        SYSCFG_DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
      \arg        SYSCFG_PA11_REMAP_PA12: remap PA11 PA12
    \param[out] none
    \retval     none
*/
void syscfg_dma_remap_enable(uint32_t syscfg_dma_remap)
{
    SYSCFG_CFG0 |= syscfg_dma_remap;
}

/*!
    \brief      disable the DMA channels remapping
    \param[in]  syscfg_dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_DMA_REMAP_TIMER16: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        SYSCFG_DMA_REMAP_TIMER15: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        SYSCFG_DMA_REMAP_USART0RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        SYSCFG_DMA_REMAP_USART0TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        SYSCFG_DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
      \arg        SYSCFG_PA11_REMAP_PA12: remap PA11 PA12
    \param[out] none
    \retval     none
*/
void syscfg_dma_remap_disable(uint32_t syscfg_dma_remap)
{
    SYSCFG_CFG0 &= ~syscfg_dma_remap;
}

/*!
    \brief      enable PBx(x=6,7,9) high current capability
    \param[in]  syscfg_high_current  
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_PB6_HIGH_CURRENT: PB6 pin high current capability enable
      \arg        SYSCFG_PB7_HIGH_CURRENT: PB7 pin high current capability enable
      \arg        SYSCFG_PB9_HIGH_CURRENT: PB9 pin high current capability enable
    \param[out] none
    \retval     none
*/
void syscfg_high_current_enable(uint32_t syscfg_high_current)
{
    SYSCFG_CFG0 |= syscfg_high_current;
}

/*!
    \brief      disable PBx(x=6,7,9) high current capability
    \param[in]  syscfg_high_current 
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_PB6_HIGH_CURRENT: PB6 pin high current capability enable
      \arg        SYSCFG_PB7_HIGH_CURRENT: PB7 pin high current capability enable
      \arg        SYSCFG_PB9_HIGH_CURRENT: PB9 pin high current capability enable
    \param[out] none
    \retval     none
*/
void syscfg_high_current_disable(uint32_t syscfg_high_current)
{
    SYSCFG_CFG0 &= syscfg_high_current;
}

/*!
    \brief      enable VREF function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void syscfg_vref_enable(void)
{
    SYSCFG_CFG2 |= SYSCFG_CFG2_VREF_EN;
}

/*!
    \brief      disable VREF function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void syscfg_vref_disable(void)
{
    SYSCFG_CFG2 &= ~SYSCFG_CFG2_VREF_EN;
}

/*!
    \brief      configure the GPIO pin reset mode
    \param[in]  syscfg_pin_reset_mode: specifies the GPIO pin reset mode.
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_PIN_NRST: GPIO pin configuration will retain state across any reset event except for the POR event
      \arg        SYSCFG_PIN_RST: GPIO pin configuration is reset when any reset event occurs 
    \param[out] none
    \retval     none
*/
void syscfg_pin_reset_mode_config(uint32_t syscfg_pin_reset_mode)
{ 
    uint32_t reg;
    
    reg = SYSCFG_CFG2;
    /* reset the ENET_PHY_SEL bit and set according to syscfg_enet_phy_interface */
    reg &= ~SYSCFG_CFG2_PIN_RSTMD;
    SYSCFG_CFG2 = (reg | syscfg_pin_reset_mode);
}

/*!
    \brief      configure the GPIO pin as EXTI Line
    \param[in]  exti_port: specify the GPIO port used in EXTI
                only one parameter can be selected which is shown as below:
      \arg        EXTI_SOURCE_GPIOx(x = A,B,C,F): EXTI GPIO port
    \param[in]  exti_pin: specify the EXTI line 
                only one parameter can be selected which is shown as below:
      \arg        EXTI_SOURCE_PINx(GPIOA x = 0..15,GPIOB x = 0..15,GPIOC x = 13..15,GPIOF x = 0.1.6.7): EXTI GPIO pin
    \param[out] none
    \retval     none
*/
void syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
    uint32_t clear_exti_mask = ~((uint32_t)EXTI_SS_MASK << (EXTI_SS_MSTEP(exti_pin)));
    uint32_t config_exti_mask = ((uint32_t)exti_port) << (EXTI_SS_MSTEP(exti_pin));

    switch(exti_pin / EXTI_SS_JSTEP){
    case EXTISS0:
        /* clear EXTI source line(0..3) */
        SYSCFG_EXTISS0 &= clear_exti_mask;
        /* configure EXTI soure line(0..3) */
        SYSCFG_EXTISS0 |= config_exti_mask;
        break;
    case EXTISS1:
        /* clear EXTI soure line(4..7) */
        SYSCFG_EXTISS1 &= clear_exti_mask;
        /* configure EXTI soure line(4..7) */
        SYSCFG_EXTISS1 |= config_exti_mask;
        break;
    case EXTISS2:
        /* clear EXTI soure line(8..11) */
        SYSCFG_EXTISS2 &= clear_exti_mask;
        /* configure EXTI soure line(8..11) */
        SYSCFG_EXTISS2 |= config_exti_mask;
        break;
    case EXTISS3:
        /* clear EXTI soure line(12..15) */
        SYSCFG_EXTISS3 &= clear_exti_mask;
        /* configure EXTI soure line(12..15) */
        SYSCFG_EXTISS3 |= config_exti_mask;
        break;
    default:
        break;
    }
}

/*!
    \brief      connect TIMER0/14/15/16 break input to the selected parameter
    \param[in]  syscfg_lock: Specify the parameter to be connected
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_LOCK_LOCKUP: Cortex-M23 lockup output connected to the break input
      \arg        SYSCFG_LOCK_SRAM_PARITY_ERROR: SRAM_PARITY check error connected to the break input
      \arg        SYSCFG_LOCK_LVD: LVD interrupt connected to the break input
    \param[out] none
    \retval     none
*/
void syscfg_lock_config(uint32_t syscfg_lock)
{
    SYSCFG_CFG2 |= syscfg_lock;
}

/*!
    \brief      set the wait state counter value
    \param[in]  irq_latency: IRQ_LATENCY value (0x00 - 0xFF)
    \param[out] none
    \retval     none
*/
void irq_latency_set(uint8_t irq_latency)
{
    uint32_t reg;
    
    reg = SYSCFG_CPU_IRQ_LAT &(~(uint32_t)SYSCFG_CPU_IRQ_LAT_IRQ_LATENCY);
    reg |= (uint32_t)(IRQ_LATENCY(irq_latency));
    
    SYSCFG_CPU_IRQ_LAT = (uint32_t)reg;
}
/*!
    \brief      check if the specified flag in SYSCFG_CFG2 is set or not.
    \param[in]  syscfg_flag: specify the flag in SYSCFG_CFG2 to check.
      \arg        SYSCFG_SRAM_PCEF: SRAM parity check error flag.
    \param[out] none
    \retval     the syscfg_flag state returned (SET or RESET).
  */
FlagStatus syscfg_flag_get(uint32_t syscfg_flag)
{
    if((SYSCFG_CFG2 & syscfg_flag) != (uint32_t)RESET){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear the flag in SYSCFG_CFG2 by writing 1.
    \param[in]  syscfg_flag: Specify the flag in SYSCFG_CFG2 to clear.
      \arg        SYSCFG_SRAM_PCEF: SRAM parity check error flag.
    \param[out] none
    \retval     none
*/
void syscfg_flag_clear(uint32_t syscfg_flag)
{
    SYSCFG_CFG2 |= (uint32_t) syscfg_flag;
}
