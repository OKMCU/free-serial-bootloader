/*!
    \file    gd32e232_fmc.c
    \brief   FMC driver
    
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

#include "gd32e232_fmc.h"

/* FMC register bit offset */
#define FMC_OBSTAT_USER_OFFSET            ((uint32_t)8U)
#define FMC_OBSTAT_DATA_OFFSET            ((uint32_t)16U)

/* write option byte */
static void ob_write(option_byte_struct* ob_struct, option_byte_enum ob_num);

/*!
    \brief      unlock the main FMC operation
                it is better to used in pairs with fmc_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_unlock(void)
{
    if((RESET != (FMC_CTL & FMC_CTL_LK))){
        /* write the FMC key */
        FMC_KEY = UNLOCK_KEY0;
        FMC_KEY = UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the main FMC operation
                it is better to used in pairs with fmc_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_lock(void)
{
    /* set the LK bit*/
    FMC_CTL |= FMC_CTL_LK;
}

/*!
    \brief      set the wait state counter value
    \param[in]  wscnt: wait state counter value
      \arg        WS_WSCNT_0: 0 wait state added
      \arg        WS_WSCNT_1: 1 wait state added
      \arg        WS_WSCNT_2: 2 wait state added
    \param[out] none
    \retval     none
*/
void fmc_wscnt_set(uint8_t wscnt)
{
    uint32_t reg;
    
    reg = FMC_WS;
    /* set the wait state counter value */
    reg &= ~FMC_WS_WSCNT;
    FMC_WS = (reg | wscnt);
}

/*!
    \brief      pre-fetch enable
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_enable(void)
{
    FMC_WS |= FMC_WS_PFEN;
}

/*!
    \brief      pre-fetch disable
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_disable(void)
{
    FMC_WS &= ~FMC_WS_PFEN;
}

/*!
    \brief      erase page
    \param[in]  page_address: target page start address
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum fmc_page_erase(uint32_t page_address)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
  
    if(FMC_READY == fmc_state){ 
        /* start page erase */
        FMC_CTL |= FMC_CTL_PER;
        FMC_ADDR = page_address;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        /* reset the PER bit */
        FMC_CTL &= ~FMC_CTL_PER;
    }
    
    /* return the FMC state  */
    return fmc_state;
}

/*!
    \brief      erase whole chip
    \param[in]  none
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum fmc_mass_erase(void)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
  
    if(FMC_READY == fmc_state){ 
        /* start chip erase */
        FMC_CTL |= FMC_CTL_MER; 
        FMC_CTL |= FMC_CTL_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MER bit */
        FMC_CTL &= ~FMC_CTL_MER;
    }
    
    /* return the fmc state  */
    return fmc_state;
}

/*!
    \brief      program a double word at the corresponding address in main flash, this 
                function also applies to OTP(address 0x1FFF_7000~0x1FFF_73FF) programming
    \param[in]  address: address to program
    \param[in]  data: double word to program
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data)
{ 
    uint32_t data0, data1;
    
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    data0 = (uint32_t)(data & 0xFFFFFFFFU);
    data1 = (uint32_t)((data>>32U) & 0xFFFFFFFFU);
    
    /* configure program width */
    if(FMC_READY == fmc_state){ 
        /* set the PG bit to start program */
        FMC_CTL |= FMC_CTL_PG; 
        REG32(address) = data0;
        REG32(address+4U) = data1;
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        /* reset the PG bit */
        FMC_CTL &= ~FMC_CTL_PG; 
    }
  
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      unlock the option byte operation
                it is better to used in pairs with ob_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_unlock(void)
{
    if(RESET == (FMC_CTL & FMC_CTL_OBWEN)){
        /* write the FMC key */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
    }
    /* wait until OBWEN bit is set by hardware */
    while(RESET == (FMC_CTL & FMC_CTL_OBWEN)){
    }
}

/*!
    \brief      lock the option byte operation
                it is better to used in pairs with ob_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_lock(void)
{
    /* reset the OBWE bit */
    FMC_CTL &= ~FMC_CTL_OBWEN;
}

/*!
    \brief      reload the option byte and generate a system reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_reset(void)
{
    /* set the OBRLD bit */
    FMC_CTL |= FMC_CTL_OBRLD;
}

/*!
    \brief      erase the option byte
                programmer must ensure FMC & option byte are both unlocked before calling this function
    \param[in]  none
    \param[out] none
    \retval     fmc state
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum ob_erase(void)
{
    option_byte_struct ob_struct;

    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
    ob_struct.ob_spc = OB_SPC;
    ob_struct.ob_user = 0xFFFF;
    ob_struct.ob_data = 0xFFFFFFFF;
    
    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_READY == fmc_state){
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
       
            /* set the OBPG bit */
            FMC_CTL |= FMC_CTL_OBPG;
            
            /* restore the last get option byte security protection code */
            ob_write(&ob_struct, DOUBLEWORD_SPC_USER_DATA);
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
 
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }else{
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
            
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }  
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      write option byte
    \param[in]  ob_struct: option byte structure 
                            and the member values are shown as below:
                  ob_spc: value of spc in option byte
                  ob_user: value of user in option byte
                  ob_data: value of data in option byte
                  ob_wp: value of wp in option byte
    \param[in]  ob_num: refer to rcu_periph_enum
      \arg        DOUBLEWORD_SPC_USER_DATA: first 64 bits of option byte, include SPC, USER, DATA
      \arg        DOUBLEWORD_WP: second 64 bits of option byte, include WP
    \param[out] none
    \retval     none
*/
static void ob_write(option_byte_struct* ob_struct, option_byte_enum ob_num)
{
    uint32_t ob_val0 = 0;
    uint32_t ob_val1 = 0;
    
    /* write first 64 bits of option byte */
    if(DOUBLEWORD_SPC_USER_DATA == ob_num){
        ob_val0 = (uint32_t)(ob_struct->ob_spc); 
        ob_val0 |= (((uint32_t)ob_struct->ob_user)<<16U);
        
        ob_val1 |= ob_struct->ob_data;
        
        REG32(0x1ffff800U) = ob_val0;
        REG32(0x1ffff804U) = ob_val1;
    }else{
        /* write second 64 bits of option byte */
        ob_val0 = (uint32_t)(ob_struct->ob_wp);
        ob_val1 = 0xffffffffU;
        REG32(0x1ffff808U) = ob_val0;
        REG32(0x1ffff80cU) = ob_val1;
    }
}
    
/*!
    \brief      enable option byte write protection (OB_WP)
    \param[in]  ob_wp: write protection configuration data. Notice that set the
                bit to 1 if you want to protect the corresponding pages.
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum ob_write_protection_enable(uint16_t ob_wp)
{
    option_byte_struct ob_struct;
    uint32_t ob_wrp_val = 0U;

    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
    ob_struct.ob_spc = OB_SPC;
    ob_struct.ob_user = OB_USER;
    ob_struct.ob_data = OB_DATA;
    ob_struct.ob_wp = OB_WP;

    ob_wp = (uint16_t)(~ob_wp);
    
    ob_wrp_val |= (uint32_t)ob_wp&0x00FFU;
    ob_wrp_val |= (((uint32_t)ob_wp&0xFF00U)>>8U) << 16U;
    
    ob_struct.ob_wp = ob_struct.ob_wp & ob_wrp_val;

    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_READY == fmc_state){
        
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
      
            /* enable the option bytes programming */
            FMC_CTL |= FMC_CTL_OBPG;
            ob_write(&ob_struct, DOUBLEWORD_WP);
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            
            ob_write(&ob_struct, DOUBLEWORD_SPC_USER_DATA);
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT); 
    
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }else{
            if(FMC_TOERR != fmc_state){
                /* reset the OBER bit */
                FMC_CTL &= ~FMC_CTL_OBER;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure security protection
    \param[in]  ob_spc: specify security protection code
      \arg        FMC_NSPC: no security protection
      \arg        FMC_LSPC: low security protection
      \arg        FMC_HSPC: high security protection
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum ob_security_protection_config(uint16_t ob_spc)
{
    option_byte_struct ob_struct;
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    ob_struct.ob_spc = ob_spc;
    ob_struct.ob_user = OB_USER;
    ob_struct.ob_data = OB_DATA;
    ob_struct.ob_wp = OB_WP;
    
    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_READY == fmc_state){
        
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
      
            /* enable the option bytes programming */
            FMC_CTL |= FMC_CTL_OBPG;
       
            ob_write(&ob_struct, DOUBLEWORD_SPC_USER_DATA);
            ob_write(&ob_struct, DOUBLEWORD_WP);
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT); 
    
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }else{
            if(FMC_TOERR != fmc_state){
                /* reset the OBER bit */
                FMC_CTL &= ~FMC_CTL_OBER;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program the FMC user option byte
                this function can only clear the corresponding bits to be 0 rather than 1.
                the function ob_erase is used to set all the bits to be 1.
    \param[in]  ob_user: user option byte
                one or more parameters (bitwise AND) can be selected which are shown as below:
      \arg        OB_FWDGT_HW: hardware free watchdog timer
      \arg        OB_DEEPSLEEP_RST: no reset when entering deepsleep mode
      \arg        OB_STDBY_RST: no reset when entering standby mode
      \arg        OB_BOOT1_SET_1: BOOT1 bit is 1
      \arg        OB_VDDA_DISABLE: disable VDDA monitor
      \arg        OB_SRAM_PARITY_ENABLE: enable sram parity check
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum ob_user_write(uint8_t ob_user)
{
    option_byte_struct ob_struct;
    /* check whether FMC is ready or not */
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
    ob_struct.ob_spc = OB_SPC;
    ob_struct.ob_user = (uint16_t)ob_user;
    ob_struct.ob_data = OB_DATA;
    ob_struct.ob_wp = OB_WP;
    
    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_READY == fmc_state){
        
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
      
            /* enable the option bytes programming */
            FMC_CTL |= FMC_CTL_OBPG;
       
            ob_write(&ob_struct, DOUBLEWORD_SPC_USER_DATA);
            ob_write(&ob_struct, DOUBLEWORD_WP);
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT); 
    
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }else{
            if(FMC_TOERR != fmc_state){
                /* reset the OBER bit */
                FMC_CTL &= ~FMC_CTL_OBER;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program the FMC data option byte
    \param[in]  data: the data to be programmed, OB_DATA[0:15]
    \param[out] none
    \retval     fmc_state: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum ob_data_program(uint16_t data)
{
    option_byte_struct ob_struct;
    
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    uint32_t val = 0U;
    
    ob_struct.ob_spc = OB_SPC;
    ob_struct.ob_user = OB_USER;
    ob_struct.ob_wp = OB_WP;
    val |= (uint32_t)data&0x00FFU;
    val |= (((uint32_t)data&0xFF00U)>>8U) << 16U;
    ob_struct.ob_data = val;
    
    if(FMC_READY == fmc_state){
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        if(FMC_READY == fmc_state){
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
            /* set the OBPG bit */
            FMC_CTL |= FMC_CTL_OBPG; 
            ob_write(&ob_struct, DOUBLEWORD_SPC_USER_DATA);
            ob_write(&ob_struct, DOUBLEWORD_WP);
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }else{
            if(FMC_TOERR != fmc_state){
                /* reset the OBER bit */
                FMC_CTL &= ~FMC_CTL_OBER;
            }
        }
    }
        
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get OB_USER in register FMC_OBSTAT
    \param[in]  none
    \param[out] none
    \retval     ob_user
*/
uint8_t ob_user_get(void)
{
    return (uint8_t)(FMC_OBSTAT >> FMC_OBSTAT_USER_OFFSET);
}

/*!
    \brief      get OB_DATA in register FMC_OBSTAT
    \param[in]  none
    \param[out] none
    \retval     ob_data
*/
uint16_t ob_data_get(void)
{
    return (uint16_t)(FMC_OBSTAT >> FMC_OBSTAT_DATA_OFFSET);
}

/*!
    \brief      get the FMC option byte write protection (OB_WP) in register FMC_WP
    \param[in]  none
    \param[out] none
    \retval     OB_WP
*/
uint16_t ob_write_protection_get(void)
{
    return (uint16_t)(FMC_WP);
}

/*!
    \brief      get the value of FMC option byte security protection level (PLEVEL) in FMC_OBSTAT register
    \param[in]  none
    \param[out] none
    \retval     the value of PLEVEL
*/
uint32_t ob_obstat_plevel_get(void)
{
    return (FMC_OBSTAT & (FMC_OBSTAT_PLEVEL_BIT0 | FMC_OBSTAT_PLEVEL_BIT1));
}

/* FMC interrupts and flags management functions */
/*!
    \brief      enable FMC interrupt
    \param[in]  interrupt: the FMC interrupt source
      \arg        FMC_INTEN_END: FMC end of operation interrupt
      \arg        FMC_INTEN_ERR: FMC error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_enable(uint32_t interrupt)
{
    FMC_CTL |= interrupt;
}

/*!
    \brief      disable FMC interrupt
    \param[in]  interrupt: the FMC interrupt source
      \arg        FMC_INTEN_END: FMC end of operation interrupt
      \arg        FMC_INTEN_ERR: FMC error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_disable(uint32_t interrupt)
{
    FMC_CTL &= ~(uint32_t)interrupt;
}

/*!
    \brief      get flag set or reset
    \param[in]  flag: check FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_FLAG_BUSY: FMC busy flag
      \arg        FMC_FLAG_PGERR: FMC programming error flag
      \arg        FMC_FLAG_PGAERR: FMC program alignment error flag
      \arg        FMC_FLAG_WPERR: FMC write protection error flag
      \arg        FMC_FLAG_END: FMC end of programming flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;

    if(FMC_STAT & flag){
        status = SET;
    }
    /* return the state of corresponding FMC flag */
    return status; 
}

/*!
    \brief      clear the FMC pending flag by writing 1
    \param[in]  flag: clear FMC flag
                one or more parameters can be selected which is shown as below:
      \arg        FMC_FLAG_PGERR: FMC programming error flag
      \arg        FMC_FLAG_PGAERR: FMC program alignment error flag
      \arg        FMC_FLAG_WPERR: FMC write protection error flag
      \arg        FMC_FLAG_END: FMC end of programming flag
    \param[out] none
    \retval     none
*/
void fmc_flag_clear(uint32_t flag)
{
    /* clear the flags */
    FMC_STAT = flag;
}

/*!
    \brief      get intrrupt flag set or reset
    \param[in]  flag: check FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_FLAG_PGERR: FMC programming error flag
      \arg        FMC_INT_FLAG_PGAERR: FMC program alignment error flag
      \arg        FMC_INT_FLAG_WPERR: FMC write protection error flag
      \arg        FMC_INT_FLAG_END: FMC end of programming flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_interrupt_flag_get(uint32_t int_flag)
{
    uint32_t intenable = 0U, flagstatus = 0U;
    
    if(FMC_INT_FLAG_END == int_flag){
        /* get the interrupt enable bit status */
        intenable = FMC_CTL & FMC_INTEN_END;
        /* get the corresponding flag bit status */
        flagstatus = FMC_STAT & int_flag;
        if(intenable && flagstatus){
            return SET;
        }else{
            return RESET;
        }
    }else{
        /* get the interrupt enable bit status */
        intenable = FMC_CTL & FMC_INTEN_ERR;
        /* get the corresponding flag bit status */
        flagstatus = FMC_STAT & int_flag;
        if(intenable && flagstatus){
            return SET;
        }else{
            return RESET;
        }
    }
}

/*!
    \brief      clear the FMC interrupt pending flag by writing 1
    \param[in]  flag: clear FMC flag
                one or more parameters can be selected which is shown as below:
      \arg        FMC_INT_FLAG_PGERR: FMC programming error flag
      \arg        FMC_INT_FLAG_PGAERR: FMC program alignment error flag
      \arg        FMC_INT_FLAG_WPERR: FMC write protection error flag
      \arg        FMC_INT_FLAG_END: FMC end of programming flag
    \param[out] none
    \retval     none
*/
void fmc_interrupt_flag_clear(uint32_t int_flag)
{
    /* clear the flags */
    FMC_STAT = int_flag;
}

/*!
    \brief      get the FMC state
    \param[in]  none
    \param[out] none
    \retval     fmc_state
*/
fmc_state_enum fmc_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;
  
    if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_BUSY)){
        fmc_state = FMC_BUSY;
    }else{
        if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_WPERR)){
            fmc_state = FMC_WPERR;
        }else{
            if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_PGERR)){
                fmc_state = FMC_PGERR; 
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC is ready or not
    \param[in]  timeout: timeout count
    \param[out] none
    \retval     fmc_state
*/
fmc_state_enum fmc_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;
  
    /* wait for FMC ready */
    do{
        /* get FMC state */
        fmc_state = fmc_state_get();
        timeout--;
    }while((FMC_BUSY == fmc_state) && (0U != timeout));
  
    if(FMC_BUSY == fmc_state){
        fmc_state = FMC_TOERR;
    }
    /* return the FMC state */
    return fmc_state;
}
