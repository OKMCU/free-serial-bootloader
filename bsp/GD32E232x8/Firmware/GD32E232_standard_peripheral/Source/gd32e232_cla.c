/*!
    \file    gd32e232_cla.c
    \brief   CLA driver
    
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

#include "gd32e232_cla.h"

/*!
    \brief      reset CLA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cla_deinit(void)
{
    rcu_periph_reset_enable(RCU_CLARST);
    rcu_periph_reset_disable(RCU_CLARST);
}

/*!
    \brief      enable CLA
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void cla_enable(cla_enum cla_periph)
{
    switch(cla_periph){
        case CLA0:
             CLA_GCTL |= CLA_GCTL_CLA0EN;
             break;
        case CLA1:
             CLA_GCTL |= CLA_GCTL_CLA1EN;
             break;
        case CLA2:
             CLA_GCTL |= CLA_GCTL_CLA2EN;
             break;
        case CLA3:
             CLA_GCTL |= CLA_GCTL_CLA3EN;
             break;
        default: 
             break;
    }
}

/*!
    \brief      disable CLA
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void cla_disable(cla_enum cla_periph)
{
    switch(cla_periph){
        case CLA0:
             CLA_GCTL &= (~CLA_GCTL_CLA0EN);
             break;
        case CLA1:
             CLA_GCTL &= (~CLA_GCTL_CLA1EN);
             break;
        case CLA2:
             CLA_GCTL &= (~CLA_GCTL_CLA2EN);
             break;
        case CLA3:
             CLA_GCTL &= (~CLA_GCTL_CLA3EN);
             break;
        default: 
             break;
    }
}

/*!
    \brief      get CLA output state
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[out] none
    \retval     cla_outputstatus_enum: CLA_OUTPUT_HIGH or CLA_OUTPUT_LOW
*/
cla_outputstatus_enum cla_output_state_get(cla_enum cla_periph)
{
    uint32_t state;
    uint32_t returnval;
    
    /* read cla status register */
    state = CLA_STAT;
    
    switch(cla_periph){
        case CLA0:
             returnval = (state & CLA_STAT_CLA0OUT);
             break;
        case CLA1:
             returnval = ((state & CLA_STAT_CLA1OUT)>>1);
             break;
        case CLA2:
             returnval = ((state & CLA_STAT_CLA2OUT)>>2);
             break;
        case CLA3:
             returnval = ((state & CLA_STAT_CLA3OUT)>>3);
             break;
        default: 
             break;
    }
    if(CLA_OUTPUT_HIGH == returnval){
        return CLA_OUTPUT_HIGH;
    }else{
        return CLA_OUTPUT_LOW;
    }
}

/*!
    \brief      configure multiplexer input
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[in]  mux: multiplexer
                only one parameter can be selected which is shown as below:
                MUX0: multiplexer 0
                MUX1: multiplexer 1
    \param[in]  input: input of multiplexer

     MUX0 input selection:
     only one parameter can be selected which is shown as below:
     --------------------------------------------------------------------------------------------------------------------------
     | MUX0[3:0]|      CLA0MUX0             |      CLA1MUX0             |      CLA2MUX0             |     CLA3MUX0            |
     --------------------------------------------------------------------------------------------------------------------------
     |   0000   | CLA0MUX0_CLA0_ASYNC_OUT   | CLA0MUX0_CLA3_ASYNC_OUT   | CLA0MUX0_CLA3_ASYNC_OUT   | CLA0MUX0_CLA3_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------
     |   0001   | CLA1MUX0_CLA1_ASYNC_OUT   | CLA1MUX0_CLA1_ASYNC_OUT   | CLA1MUX0_CLA1_ASYNC_OUT   | CLA1MUX0_CLA1_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------
     |   0010   | CLA2MUX0_CLA2_ASYNC_OUT   | CLA2MUX0_CLA2_ASYNC_OUT   | CLA2MUX0_CLA2_ASYNC_OUT   | CLA2MUX0_CLA2_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------
     |   0011   | CLA3MUX0_CLA3_ASYNC_OUT   | CLA3MUX0_CLA3_ASYNC_OUT   | CLA3MUX0_CLA3_ASYNC_OUT   | CLA3MUX0_CLA3_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------------------------
     |   0100   | CLA0MUX0_TIMER1_TRGO      | CLA1MUX0_TIMER1_TRGO      | CLA2MUX0_TIMER1_TRGO      | CLA3MUX0_TIMER1_TRGO    |
     --------------------------------------------------------------------------------------------------------------------------
     |   0101   | CLA0MUX0_TIMER0_CH0       | CLA1MUX0_TIMER0_CH0       | CLA2MUX0_TIMER0_CH1       | CLA3MUX0_TIMER0_CH2     |
     --------------------------------------------------------------------------------------------------------------------------
     |   0110   | CLA0MUX0_TIMER0_CH1       | CLA1MUX0_TIMER0_CH3       | CLA2MUX0_TIMER0_CH3       | CLA3MUX0_TIMER1_CH0     |
     --------------------------------------------------------------------------------------------------------------------------
     |   0111   | CLA0MUX0_TIMER0_CH2       | CLA1MUX0_TIMER1_CH0       | CLA2MUX0_TIMER1_CH1       | CLA3MUX0_TIMER1_CH1     |
     --------------------------------------------------------------------------------------------------------------------------
     |   1000   | CLA0MUX0_CLAIN0           | CLA1MUX0_CLAIN4           | CLA2MUX0_CLAIN0           | CLA3MUX0_CLAIN2         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1001   | CLA0MUX0_CLAIN2           | CLA1MUX0_CLAIN5           | CLA2MUX0_CLAIN1           | CLA3MUX0_CLAIN3         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1010   | CLA0MUX0_CLAIN4           | CLA1MUX0_CLAIN8           | CLA2MUX0_CLAIN8           | CLA3MUX0_CLAIN6         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1011   | CLA0MUX0_CLAIN6           | CLA1MUX0_CLAIN10          | CLA2MUX0_CLAIN9           | CLA3MUX0_CLAIN7         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1100   | CLA0MUX0_CLAIN8           | CLA1MUX0_CLAIN12          | CLA2MUX0_CLAIN14          | CLA3MUX0_CLAIN10        |
     --------------------------------------------------------------------------------------------------------------------------
     |   1101   | CLA0MUX0_CLAIN10          | CLA1MUX0_CLAIN13          | CLA2MUX0_CLAIN15          | CLA3MUX0_CLAIN11        |
     --------------------------------------------------------------------------------------------------------------------------
     |   1110   | CLA0MUX0_CLAIN12          | CLA1MUX0_CLAIN16          | CLA2MUX0_CLAIN16          | CLA3MUX0_CLAIN18        |
     --------------------------------------------------------------------------------------------------------------------------
     |   1111   | CLA0MUX0_CLAIN14          | CLA1MUX0_CLAIN18          | CLA2MUX0_CLAIN17          | CLA3MUX0_CLAIN19        |
     --------------------------------------------------------------------------------------------------------------------------
     MUX1 input selection:
     only one parameter can be selected which is shown as below:
     --------------------------------------------------------------------------------------------------------------------------
     | MUX1[3:0]|      CLA0MUX1             |      CLA1MUX1             |       CLA2MUX1            |     CLA3MUX1            |
     --------------------------------------------------------------------------------------------------------------------------
     |   0000   | CLA0MUX1_CLA0_ASYNC_OUT   | CLA0MUX1_CLA0_ASYNC_OUT   | CLA0MUX1_CLA0_ASYNC_OUT   | CLA0MUX1_CLA0_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------------------------
     |   0001   | CLA1MUX1_CLA1_ASYNC_OUT   | CLA1MUX1_CLA1_ASYNC_OUT   | CLA1MUX1_CLA1_ASYNC_OUT   | CLA1MUX1_CLA1_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------------------------
     |   0010   | CLA2MUX1_CLA2_ASYNC_OUT   | CLA2MUX1_CLA2_ASYNC_OUT   | CLA2MUX1_CLA2_ASYNC_OUT   | CLA2MUX1_CLA2_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------------------------
     |   0011   | CLA3MUX1_CLA3_ASYNC_OUT   | CLA3MUX1_CLA3_ASYNC_OUT   | CLA3MUX1_CLA3_ASYNC_OUT   | CLA3MUX1_CLA3_ASYNC_OUT |
     --------------------------------------------------------------------------------------------------------------------------
     |   0100   | CLA0MUX1_ADC_CONV         | CLA1MUX1_ADC_CONV         | CLA2MUX1_ADC_CONV         | CLA3MUX1_ADC_CONV       |
     --------------------------------------------------------------------------------------------------------------------------
     |   0101   | CLA0MUX1_TIMER0_CH3       | CLA1MUX1_TIMER0_CH1       | CLA2MUX1_TIMER0_CH0       | CLA3MUX1_TIMER0_CH0     |
     --------------------------------------------------------------------------------------------------------------------------
     |   0110   | CLA0MUX1_TIMER1_CH0       | CLA1MUX1_TIMER0_CH2       | CLA2MUX1_TIMER0_CH2       | CLA3MUX1_TIMER0_CH1     |
     --------------------------------------------------------------------------------------------------------------------------
     |   0111   | CLA0MUX1_TIMER2_CH1       | CLA1MUX1_TIMER2_CH1       | CLA2MUX1_TIMER1_CH0       | CLA3MUX1_TIMER0_CH3     |
     --------------------------------------------------------------------------------------------------------------------------
     |   1000   | CLA0MUX1_CLAIN1           | CLA1MUX1_CLAIN6           | CLA2MUX1_CLAIN2           | CLA3MUX1_CLAIN0         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1001   | CLA0MUX1_CLAIN3           | CLA1MUX1_CLAIN7           | CLA2MUX1_CLAIN3           | CLA3MUX1_CLAIN1         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1010   | CLA0MUX1_CLAIN5           | CLA1MUX1_CLAIN9           | CLA2MUX1_CLAIN10          | CLA3MUX1_CLAIN4         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1011   | CLA0MUX1_CLAIN6           | CLA1MUX1_CLAIN11          | CLA2MUX1_CLAIN11          | CLA3MUX1_CLAIN5         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1100   | CLA0MUX1_CLAIN9           | CLA1MUX1_CLAIN14          | CLA2MUX1_CLAIN12          | CLA3MUX1_CLAIN8         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1101   | CLA0MUX1_CLAIN11          | CLA1MUX1_CLAIN15          | CLA2MUX1_CLAIN13          | CLA3MUX1_CLAIN9         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1110   | CLA0MUX1_CLAIN11          | CLA1MUX1_CLAIN15          | CLA2MUX1_CLAIN13          | CLA3MUX1_CLAIN9         |
     --------------------------------------------------------------------------------------------------------------------------
     |   1111   | CLA0MUX1_CLAIN13          | CLA1MUX1_CLAIN17          | CLA2MUX1_CLAIN18          | CLA3MUX1_CLAIN16        |
     --------------------------------------------------------------------------------------------------------------------------
    \param[out] none
    \retval     none
*/
void cla_multiplexer_input_config(cla_enum cla_periph, cla_mux_enum mux, uint32_t input)
{
    if(MUX0 == mux){
        CLA_MUXS(cla_periph) &= (~CLA_MUXS_MUX0);
        CLA_MUXS(cla_periph) |= input;
    }else{
        CLA_MUXS(cla_periph) &= (~CLA_MUXS_MUX1);
        CLA_MUXS(cla_periph) |= input;
    }
}

/*!
    \brief      configure CLA LUT control register value
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[in]  value: the value need to set
    \param[out] none
    \retval     none
*/
void cla_lut_control_config(cla_enum cla_periph, uint8_t lutctl_value)
{
    CLA_LUTCTL(cla_periph) &= (~CLA_LUT);
    CLA_LUTCTL(cla_periph) |= LUTCTL(lutctl_value);
}

/*!
    \brief      configure CLA output
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[in]  output: output of CLA
                only one parameter can be selected which is shown as below:
                FLIP_FLOP_OUTPUT: flip-flop output is selected as CLA output
                LUT_RESULT: LUT result is selected as CLA output
    \param[out] none
    \retval     none
*/
void cla_output_config(cla_enum cla_periph, uint32_t output)
{
    CLA_CTL(cla_periph) &= (~CLA_CTL_OSEL);
    CLA_CTL(cla_periph) |= output;
}

/*!
    \brief      enable CLA output
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void cla_output_enable(cla_enum cla_periph)
{
    CLA_CTL(cla_periph) |= CLA_CTL_OEN;
}

/*!
    \brief      disable CLA output
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void cla_output_disable(cla_enum cla_periph)
{
    CLA_CTL(cla_periph) &= (~CLA_CTL_OEN);
}

/*!
    \brief      reset the flip-flop output asynchronuously
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void cla_flip_flop_output_reset(cla_enum cla_periph)
{
    CLA_CTL(cla_periph) |= CLA_CTL_FFRST;
}

/*!
    \brief      configure clock polarity of flip-flop
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[in]  polarity: clock polarity of flip-flop
                only one parameter can be selected which is shown as below:
                CLA_CLOCKPOLARITY_POSEDGE: clock posedge is valid
                CLA_CLOCKPOLARITY_NEGEDGE: clock negedge is valid
    \param[out] none
    \retval     none
*/
void cla_flip_flop_clockpolarity_config(cla_enum cla_periph, uint32_t polarity)
{
    CLA_CTL(cla_periph) &= (~CLA_CTL_CPOL);
    CLA_CTL(cla_periph) |= polarity;
}

/*!
    \brief      configure clock source of flip-flop 
    \param[in]  cla_periph: CLAx(x=0,1,2,3)
    \param[in]  clock_source: clock source of flip-flop
                only one parameter can be selected which is shown as below:
                PRE_CLA_LUT_RESULT: the result of the previous CLA units
                MUX0_OUTPUT: the multiplexer output of MUX0
                HCLK: HCLK
                TIMER_TRGO: TIMER_TRGO
    \param[out] none
    \retval     none
*/
void cla_flip_flop_clocksource_config(cla_enum cla_periph, uint32_t clock_source)
{
    CLA_CTL(cla_periph) &= (~CLA_CTL_CSEL);
    CLA_CTL(cla_periph) |= clock_source;
}

/*!
    \brief      check CLA flag is set or not
    \param[in]  flag: CLA flags,refer to cla_flag_enum
                only one parameter can be selected which is shown as below:
                CLA_FLAG_CLA0NF: CLA0 unit negedge flag
                CLA_FLAG_CLA0PF: CLA0 unit posedge flag
                CLA_FLAG_CLA1NF: CLA1 unit negedge flag
                CLA_FLAG_CLA1PF: CLA1 unit posedge flag
                CLA_FLAG_CLA2NF: CLA2 unit negedge flag
                CLA_FLAG_CLA2PF: CLA2 unit posedge flag
                CLA_FLAG_CLA3NF: CLA3 unit negedge flag
                CLA_FLAG_CLA3PF: CLA3 unit posedge flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus cla_flag_get(cla_flag_enum flag)
{
    uint32_t interrupt_flag_register;
    uint32_t returnval;
    
    interrupt_flag_register = CLA_INTF;
    
    switch(flag){
        case CLA_FLAG_CLA0NF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA0NF);
             break;
        case CLA_FLAG_CLA0PF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA0PF)>>1;
             break;
        case CLA_FLAG_CLA1NF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA1NF)>>2;
             break;
        case CLA_FLAG_CLA1PF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA1PF)>>3;
             break;
        case CLA_FLAG_CLA2NF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA2NF)>>4;
             break;
        case CLA_FLAG_CLA2PF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA2PF)>>5;
             break;
        case CLA_FLAG_CLA3NF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA3NF)>>6;
             break;
        case CLA_FLAG_CLA3PF:
             returnval = (interrupt_flag_register & CLA_INTF_CLA3PF)>>7;
             break;
        default: 
             break;
    }
    if(SET == returnval){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear CLA flag
    \param[in]  flag: CLA flags,refer to cla_flag_enum
                only one parameter can be selected which is shown as below:
                CLA_FLAG_CLA0NF: CLA0 unit negedge flag
                CLA_FLAG_CLA0PF: CLA0 unit posedge flag
                CLA_FLAG_CLA1NF: CLA1 unit negedge flag
                CLA_FLAG_CLA1PF: CLA1 unit posedge flag
                CLA_FLAG_CLA2NF: CLA2 unit negedge flag
                CLA_FLAG_CLA2PF: CLA2 unit posedge flag
                CLA_FLAG_CLA3NF: CLA3 unit negedge flag
                CLA_FLAG_CLA3PF: CLA3 unit posedge flag
    \param[out] none
    \retval     none
*/
void cla_flag_clear(cla_flag_enum flag)
{
    switch(flag){
        case CLA_FLAG_CLA0NF:
             CLA_INTF &= (~CLA_INTF_CLA0NF);
             break;
        case CLA_FLAG_CLA0PF:
             CLA_INTF &= (~CLA_INTF_CLA0PF);
             break;
        case CLA_FLAG_CLA1NF:
             CLA_INTF &= (~CLA_INTF_CLA1NF);
             break;
        case CLA_FLAG_CLA1PF:
             CLA_INTF &= (~CLA_INTF_CLA1PF);
             break;
        case CLA_FLAG_CLA2NF:
             CLA_INTF &= (~CLA_INTF_CLA2NF);
             break;
        case CLA_FLAG_CLA2PF:
             CLA_INTF &= (~CLA_INTF_CLA2PF);
             break;
        case CLA_FLAG_CLA3NF:
             CLA_INTF &= (~CLA_INTF_CLA3NF);
             break;
        case CLA_FLAG_CLA3PF:
             CLA_INTF &= (~CLA_INTF_CLA3PF);
             break;
        default:
            break;
    }
}

/*!
    \brief      enable CLA negedge interrupt
    \param[in]  clanie: CLA negedge interrupt enable
                only one parameter can be selected which is shown as below:
                CLA0NIE: CLA0 negedge interrupt enable
                CLA1NIE: CLA1 negedge interrupt enable
                CLA2NIE: CLA2 negedge interrupt enable
                CLA3NIE: CLA3 negedge interrupt enable
    \param[out] none
    \retval     none
*/
void cla_negedge_interrupt_enable(uint32_t clanie)
{
    CLA_INTE |= clanie;
}

/*!
    \brief      disable CLA negedge interrupt
    \param[in]  clanidis: CLA negedge interrupt disable
                only one parameter can be selected which is shown as below:
                CLA0NI_DISABLE: CLA0 negedge interrupt disable
                CLA1NI_DISABLE: CLA1 negedge interrupt disable
                CLA2NI_DISABLE: CLA2 negedge interrupt disable
                CLA3NI_DISABLE: CLA3 negedge interrupt disable
    \param[out] none
    \retval     none
*/
void cla_negedge_interrupt_disable(uint32_t clanidis)
{
    CLA_INTE &= (~clanidis);
}

/*!
    \brief      enable CLA posedge interrupt
    \param[in]  clapie: CLA posedge interrupt enable
                only one parameter can be selected which is shown as below:
                CLA0PIE: CLA0 posedge interrupt enable
                CLA1PIE: CLA1 posedge interrupt enable
                CLA2PIE: CLA2 posedge interrupt enable
                CLA3PIE: CLA3 posedge interrupt enable
    \param[out] none
    \retval     none
*/
void cla_posedge_interrupt_enable(uint32_t clapie)
{
    CLA_INTE |= clapie;
}

/*!
    \brief      disable CLA posedge interrupt
    \param[in]  clanidis: CLA posedge interrupt disable
                only one parameter can be selected which is shown as below:
                CLA0PI_DISABLE: CLA0 posedge interrupt disable
                CLA1PI_DISABLE: CLA1 posedge interrupt disable
                CLA2PI_DISABLE: CLA2 posedge interrupt disable
                CLA3PI_DISABLE: CLA3 posedge interrupt disable
    \param[out] none
    \retval     none
*/
void cla_posedge_interrupt_disable(uint32_t clapidis)
{
    CLA_INTE &= (~clapidis);
}

/*!
    \brief      check CLA interrupt flag is set or not 
    \param[in]  int_flag: CLA interrupt flags,refer to cla_interrupt_flag_enum
                only one parameter can be selected which is shown as below:
                CLA_INT_FLAG_CLA0NF: CLA0 unit negedge interrupt flag
                CLA_INT_FLAG_CLA0PF: CLA0 unit posedge interrupt flag
                CLA_INT_FLAG_CLA1NF: CLA1 unit negedge interrupt flag
                CLA_INT_FLAG_CLA1PF: CLA1 unit posedge interrupt flag
                CLA_INT_FLAG_CLA2NF: CLA2 unit negedge interrupt flag
                CLA_INT_FLAG_CLA2PF: CLA2 unit posedge interrupt flag
                CLA_INT_FLAG_CLA3NF: CLA3 unit negedge interrupt flag
                CLA_INT_FLAG_CLA3PF: CLA3 unit posedge interrupt flag
    \param[out] none
    \retval     none
*/
FlagStatus cla_interrupt_flag_get(cla_interrupt_flag_enum int_flag)
{
    uint32_t cla_inte = CLA_INTE;
    uint32_t cla_intf = CLA_INTF;
    
    switch(int_flag){
        case CLA_INT_FLAG_CLA0NF:
             cla_inte = cla_inte & CLA_INTE_CLAONIE;
             cla_intf = cla_intf & CLA_INTF_CLA0NF;
             break;
        case CLA_INT_FLAG_CLA0PF:
             cla_inte = cla_inte & CLA_INTE_CLAOPIE;
             cla_intf = cla_intf & CLA_INTF_CLA0PF;
             break;
        case CLA_INT_FLAG_CLA1NF:
             cla_inte = cla_inte & CLA_INTE_CLA1NIE;
             cla_intf = cla_intf & CLA_INTF_CLA1NF;
             break;
        case CLA_INT_FLAG_CLA1PF:
             cla_inte = cla_inte & CLA_INTE_CLA1PIE;
             cla_intf = cla_intf & CLA_INTF_CLA1PF;
             break;
        case CLA_INT_FLAG_CLA2NF:
             cla_inte = cla_inte & CLA_INTE_CLA2NIE;
             cla_intf = cla_intf & CLA_INTF_CLA2NF;
             break;
        case CLA_INT_FLAG_CLA2PF:
             cla_inte = cla_inte & CLA_INTE_CLA2PIE;
             cla_intf = cla_intf & CLA_INTF_CLA2PF;
             break;
        case CLA_INT_FLAG_CLA3NF:
             cla_inte = cla_inte & CLA_INTE_CLA3NIE;
             cla_intf = cla_intf & CLA_INTF_CLA3NF;
             break;
        case CLA_INT_FLAG_CLA3PF:
             cla_inte = cla_inte & CLA_INTE_CLA3PIE;
             cla_intf = cla_intf & CLA_INTF_CLA3PF;
             break;
        default:
             break;
    }
    if((0U != cla_inte) && (0U != cla_intf)){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear CLA interrupt flag
    \param[in]  flag: CLA interrupt flags,refer to cla_interrupt_flag_enum
                only one parameter can be selected which is shown as below:
                CLA_INT_FLAG_CLA0NF: CLA0 unit negedge interrupt flag
                CLA_INT_FLAG_CLA0PF: CLA0 unit posedge interrupt flag
                CLA_INT_FLAG_CLA1NF: CLA1 unit negedge interrupt flag
                CLA_INT_FLAG_CLA1PF: CLA1 unit posedge interrupt flag
                CLA_INT_FLAG_CLA2NF: CLA2 unit negedge interrupt flag
                CLA_INT_FLAG_CLA2PF: CLA2 unit posedge interrupt flag
                CLA_INT_FLAG_CLA3NF: CLA3 unit negedge interrupt flag
                CLA_INT_FLAG_CLA3PF: CLA3 unit posedge interrupt flag
    \param[out] none
    \retval     none
*/
void cla_interrupt_flag_clear(cla_interrupt_flag_enum int_flag)
{
    switch(int_flag){
        case CLA_INT_FLAG_CLA0NF:
             CLA_INTF &= (~CLA_INTF_CLA0NF);
             break;
        case CLA_INT_FLAG_CLA0PF:
             CLA_INTF &= (~CLA_INTF_CLA0PF);
             break;
        case CLA_INT_FLAG_CLA1NF:
             CLA_INTF &= (~CLA_INTF_CLA1NF);
             break;
        case CLA_INT_FLAG_CLA1PF:
             CLA_INTF &= (~CLA_INTF_CLA1PF);
             break;
        case CLA_INT_FLAG_CLA2NF:
             CLA_INTF &= (~CLA_INTF_CLA2NF);
             break;
        case CLA_INT_FLAG_CLA2PF:
             CLA_INTF &= (~CLA_INTF_CLA2PF);
             break;
        case CLA_INT_FLAG_CLA3NF:
             CLA_INTF &= (~CLA_INTF_CLA3NF);
             break;
        case CLA_INT_FLAG_CLA3PF:
             CLA_INTF &= (~CLA_INTF_CLA3PF);
             break;
        default:
             break;
    }
}

