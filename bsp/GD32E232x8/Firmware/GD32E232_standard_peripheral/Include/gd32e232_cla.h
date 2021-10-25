/*!
    \file  gd32e232_cla.h
    \brief definitions for the CLA
    
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

#ifndef GD32E232_CLA_H
#define GD32E232_CLA_H

#include "gd32e232.h"

/*CLA definitions */
#define CLA                                CLA_BASE

/* registers definitions */
#define CLA_GCTL                           REG32((CLA) + 0x00000000U)                  /*!< CLA global control register */
#define CLA_INTE                           REG32((CLA) + 0x00000004U)                  /*!< CLA interrupt flag enable register */
#define CLA_INTF                           REG32((CLA) + 0x00000008U)                  /*!< CLA interrupt flag register */
#define CLA_STAT                           REG32((CLA) + 0x0000000CU)                  /*!< CLA status register */
#define CLA0_MUXS                          REG32((CLA) + 0x00000010U)                  /*!< CLA0 multiplexer selection register */
#define CLA1_MUXS                          REG32((CLA) + 0x0000001CU)                  /*!< CLA1 multiplexer selection register */
#define CLA2_MUXS                          REG32((CLA) + 0x00000028U)                  /*!< CLA2 multiplexer selection register */
#define CLA3_MUXS                          REG32((CLA) + 0x00000034U)                  /*!< CLA3 multiplexer selection register */
#define CLA0_LUTCTL                        REG32((CLA) + 0x00000014U)                  /*!< CLA0 LUT control register */
#define CLA1_LUTCTL                        REG32((CLA) + 0x00000020U)                  /*!< CLA1 LUT control register */
#define CLA2_LUTCTL                        REG32((CLA) + 0x0000002CU)                  /*!< CLA2 LUT control register */
#define CLA3_LUTCTL                        REG32((CLA) + 0x00000038U)                  /*!< CLA3 LUT control register */
#define CLA0_CTL                           REG32((CLA) + 0x00000018U)                  /*!< CLA0 control registor */
#define CLA1_CTL                           REG32((CLA) + 0x00000024U)                  /*!< CLA1 control register */
#define CLA2_CTL                           REG32((CLA) + 0x00000030U)                  /*!< CLA2 control register */
#define CLA3_CTL                           REG32((CLA) + 0x0000003CU)                  /*!< CLA3 control register */

#define CLA_MUXS(CLAx)                     REG32((CLA) + 0x10U + ((CLAx) * 0x0CU))     /*!< CLA multiplexer selection register */
#define CLA_LUTCTL(CLAx)                   REG32((CLA) + 0x14U + ((CLAx) * 0x0CU))     /*!< CLA LUT control register */
#define CLA_CTL(CLAx)                      REG32((CLA) + 0x18U + ((CLAx) * 0x0CU))     /*!< CLA control register */

/* bits definitions */
/* CLA_GCTL */
#define CLA_GCTL_CLA0EN                    BIT(0)                           /*!< CLA0 unit enable */
#define CLA_GCTL_CLA1EN                    BIT(1)                           /*!< CLA1 unit enable */
#define CLA_GCTL_CLA2EN                    BIT(2)                           /*!< CLA2 unit enable */
#define CLA_GCTL_CLA3EN                    BIT(3)                           /*!< CLA3 unit enable */

/* CLA_INTE */
#define CLA_INTE_CLAONIE                   BIT(0)                           /*!< CLA0 uint negedge interrupt enable */
#define CLA_INTE_CLAOPIE                   BIT(1)                           /*!< CLA0 uint posedge interrupt enable */
#define CLA_INTE_CLA1NIE                   BIT(2)                           /*!< CLA1 uint negedge interrupt enable */
#define CLA_INTE_CLA1PIE                   BIT(3)                           /*!< CLA1 uint posedge interrupt enable */
#define CLA_INTE_CLA2NIE                   BIT(4)                           /*!< CLA2 uint negedge interrupt enable */
#define CLA_INTE_CLA2PIE                   BIT(5)                           /*!< CLA2 uint posedge interrupt enable */
#define CLA_INTE_CLA3NIE                   BIT(6)                           /*!< CLA3 uint negedge interrupt enable */
#define CLA_INTE_CLA3PIE                   BIT(7)                           /*!< CLA3 uint posedge interrupt enable */

/* CLA_INTF */
#define CLA_INTF_CLA0NF                    BIT(0)                           /*!< CLA0 uint negedge flag */
#define CLA_INTF_CLA0PF                    BIT(1)                           /*!< CLA0 uint posedge flag */
#define CLA_INTF_CLA1NF                    BIT(2)                           /*!< CLA1 uint negedge flag */
#define CLA_INTF_CLA1PF                    BIT(3)                           /*!< CLA1 uint posedge flag */
#define CLA_INTF_CLA2NF                    BIT(4)                           /*!< CLA2 uint negedge flag */
#define CLA_INTF_CLA2PF                    BIT(5)                           /*!< CLA2 uint posedge flag */
#define CLA_INTF_CLA3NF                    BIT(6)                           /*!< CLA3 uint negedge flag */
#define CLA_INTF_CLA3PF                    BIT(7)                           /*!< CLA3 uint posedge flag */

/* CLA_STAT */
#define CLA_STAT_CLA0OUT                   BIT(0)                           /*!< CLA0 unit output state */
#define CLA_STAT_CLA1OUT                   BIT(1)                           /*!< CLA1 unit output state */
#define CLA_STAT_CLA2OUT                   BIT(2)                           /*!< CLA2 unit output state */
#define CLA_STAT_CLA3OUT                   BIT(3)                           /*!< CLA3 unit output state */

/* CLAx_MUXS */
#define CLA_MUXS_MUX1                      BITS(0,3)                        /*!< multiplexer 1 input selection */
#define CLA_MUXS_MUX0                      BITS(4,7)                        /*!< multiplexer 0 input selection */

/* CLAx_LUTCTL */
#define CLA_LUT                            BITS(0,7)                        /*!< LUT control */

/* CLAx_CTL */
#define CLA_CTL_CSEL                       BITS(0,1)                        /*!< clock selection */
#define CLA_CTL_CPOL                       BIT(2)                           /*!< clock polarity*/
#define CLA_CTL_FFRST                      BIT(3)                           /*!< Flip- flop output reset */
#define CLA_CTL_OEN                        BIT(6)                           /*!< output enable */
#define CLA_CTL_OSEL                       BIT(7)                           /*!< output selection */

/* constants definitions */
/* CLA flags */
typedef enum
{
    /* flags in INTF register */
    CLA_FLAG_CLA0NF,                                                        /*!< CLA0 unit negedge flag */
    CLA_FLAG_CLA0PF,                                                        /*!< CLA0 unit posedge flag */
    CLA_FLAG_CLA1NF,                                                        /*!< CLA1 unit negedge flag */
    CLA_FLAG_CLA1PF,                                                        /*!< CLA1 unit posedge flag */
    CLA_FLAG_CLA2NF,                                                        /*!< CLA2 unit negedge flag */
    CLA_FLAG_CLA2PF,                                                        /*!< CLA2 unit posedge flag */
    CLA_FLAG_CLA3NF,                                                        /*!< CLA3 unit negedge flag */
    CLA_FLAG_CLA3PF                                                         /*!< CLA3 unit posedge flag */
}cla_flag_enum;

/* CLA interrupt flags */
typedef enum
{
    /* interrupt flags in INTF register */
    CLA_INT_FLAG_CLA0NF,                                                    /*!< CLA0 unit negedge interrupt flag */
    CLA_INT_FLAG_CLA0PF,                                                    /*!< CLA0 unit posedge interrupt flag */
    CLA_INT_FLAG_CLA1NF,                                                    /*!< CLA1 unit negedge interrupt flag */
    CLA_INT_FLAG_CLA1PF,                                                    /*!< CLA1 unit posedge interrupt flag */
    CLA_INT_FLAG_CLA2NF,                                                    /*!< CLA2 unit negedge interrupt flag */
    CLA_INT_FLAG_CLA2PF,                                                    /*!< CLA2 unit posedge interrupt flag */
    CLA_INT_FLAG_CLA3NF,                                                    /*!< CLA3 unit negedge interrupt flag */
    CLA_INT_FLAG_CLA3PF                                                     /*!< CLA3 unit posedge interrupt flag */
}cla_interrupt_flag_enum;

/* CLA units */
typedef enum
{
    CLA0,
    CLA1,
    CLA2,
    CLA3
}cla_enum;

/* CLA multiplexer */
typedef enum
{
    MUX0,
    MUX1
}cla_mux_enum;

/* output state of CLAx unit */
typedef enum
{
    CLA_OUTPUT_LOW,
    CLA_OUTPUT_HIGH
}cla_outputstatus_enum;

/* negedge interrupt enable */
#define CLA0NIE                          CLA_INTE_CLAONIE                   /*!< enable CLA0 unit negedge interrupt */
#define CLA1NIE                          CLA_INTE_CLA1NIE                   /*!< enable CLA1 unit negedge interrupt */
#define CLA2NIE                          CLA_INTE_CLA2NIE                   /*!< enable CLA2 unit negedge interrupt */
#define CLA3NIE                          CLA_INTE_CLA3NIE                   /*!< enable CLA3 unit negedge interrupt */

/* negedge interrupt disable */
#define CLA0NI_DISABLE                   CLA_INTE_CLAONIE                   /*!< disable CLA0 unit negedge interrupt */
#define CLA1NI_DISABLE                   CLA_INTE_CLA1NIE                   /*!< disable CLA1 unit negedge interrupt */
#define CLA2NI_DISABLE                   CLA_INTE_CLA2NIE                   /*!< disable CLA2 unit negedge interrupt */
#define CLA3NI_DISABLE                   CLA_INTE_CLA3NIE                   /*!< disable CLA3 unit negedge interrupt */

/* posedge interrupt enable */
#define CLA0PIE                          CLA_INTE_CLAOPIE                   /*!< enable CLA0 unit  posedge interrupt */
#define CLA1PIE                          CLA_INTE_CLA1PIE                   /*!< enable CLA1 unit  posedge interrupt */
#define CLA2PIE                          CLA_INTE_CLA2PIE                   /*!< enable CLA2 unit  posedge interrupt */
#define CLA3PIE                          CLA_INTE_CLA3PIE                   /*!< enable CLA3 unit  posedge interrupt */

/* posedge interrupt disable */
#define CLA0PI_DISABLE                   CLA_INTE_CLAOPIE                   /*!< disable CLA0 unit  posedge interrupt */
#define CLA1PI_DISABLE                   CLA_INTE_CLA1PIE                   /*!< disable CLA1 unit  posedge interrupt */
#define CLA2PI_DISABLE                   CLA_INTE_CLA2PIE                   /*!< disable CLA2 unit  posedge interrupt */
#define CLA3PI_DISABLE                   CLA_INTE_CLA3PIE                   /*!< disable CLA3 unit  posedge interrupt */

/* multiplexer 0 input selection */
#define CLA_MUX0(regval)                 (BITS(4,7) & ((uint32_t)(regval) << 4))
#define CLA0MUX0_CLA0_ASYNC_OUT          CLA_MUX0(0)                        /*!< the input of CLA0MUX0 is CLA0_ASYNC_OUT */
#define CLA1MUX0_CLA0_ASYNC_OUT          CLA_MUX0(0)                        /*!< the input of CLA1MUX0 is CLA0_ASYNC_OUT */
#define CLA2MUX0_CLA0_ASYNC_OUT          CLA_MUX0(0)                        /*!< the input of CLA2MUX0 is CLA0_ASYNC_OUT */
#define CLA3MUX0_CLA0_ASYNC_OUT          CLA_MUX0(0)                        /*!< the input of CLA3MUX0 is CLA0_ASYNC_OUT */

#define CLA0MUX0_CLA1_ASYNC_OUT          CLA_MUX0(1)                        /*!< the input of CLA0MUX0 is CLA1_ASYNC_OUT */
#define CLA1MUX0_CLA1_ASYNC_OUT          CLA_MUX0(1)                        /*!< the input of CLA1MUX0 is CLA1_ASYNC_OUT */
#define CLA2MUX0_CLA1_ASYNC_OUT          CLA_MUX0(1)                        /*!< the input of CLA2MUX0 is CLA1_ASYNC_OUT */
#define CLA3MUX0_CLA1_ASYNC_OUT          CLA_MUX0(1)                        /*!< the input of CLA3MUX0 is CLA1_ASYNC_OUT */

#define CLA0MUX0_CLA2_ASYNC_OUT          CLA_MUX0(2)                        /*!< the input of CLA0MUX0 is CLA2_ASYNC_OUT */
#define CLA1MUX0_CLA2_ASYNC_OUT          CLA_MUX0(2)                        /*!< the input of CLA1MUX0 is CLA2_ASYNC_OUT */
#define CLA2MUX0_CLA2_ASYNC_OUT          CLA_MUX0(2)                        /*!< the input of CLA2MUX0 is CLA2_ASYNC_OUT */
#define CLA3MUX0_CLA2_ASYNC_OUT          CLA_MUX0(2)                        /*!< the input of CLA3MUX0 is CLA2_ASYNC_OUT */

#define CLA0MUX0_CLA3_ASYNC_OUT          CLA_MUX0(3)                        /*!< the input of CLA0MUX0 is CLA3_ASYNC_OUT */
#define CLA1MUX0_CLA3_ASYNC_OUT          CLA_MUX0(3)                        /*!< the input of CLA1MUX0 is CLA3_ASYNC_OUT */
#define CLA2MUX0_CLA3_ASYNC_OUT          CLA_MUX0(3)                        /*!< the input of CLA2MUX0 is CLA3_ASYNC_OUT */
#define CLA3MUX0_CLA3_ASYNC_OUT          CLA_MUX0(3)                        /*!< the input of CLA3MUX0 is CLA3_ASYNC_OUT */

#define CLA0MUX0_TIMER1_TRGO             CLA_MUX0(4)                        /*!< the input of CLA0MUX0 is TIMER1_TRGO */
#define CLA1MUX0_TIMER2_TRGO             CLA_MUX0(4)                        /*!< the input of CLA1MUX0 is TIMER2_TRGO */
#define CLA2MUX0_TIMER5_TRGO             CLA_MUX0(4)                        /*!< the input of CLA2MUX0 is TIMER5_TRGO */
#define CLA3MUX0_TIMER6_TRGO             CLA_MUX0(4)                        /*!< the input of CLA3MUX0 is TIMER6_TRGO */

#define CLA0MUX0_TIMER0_CH0              CLA_MUX0(5)                        /*!< the input of CLA0MUX0 is TIMER0_CH0 */
#define CLA1MUX0_TIMER0_CH0              CLA_MUX0(5)                        /*!< the input of CLA1MUX0 is TIMER0_CH0 */
#define CLA2MUX0_TIMER0_CH1              CLA_MUX0(5)                        /*!< the input of CLA2MUX0 is TIMER0_CH1 */
#define CLA3MUX0_TIMER0_CH2              CLA_MUX0(5)                        /*!< the input of CLA3MUX0 is TIMER0_CH2 */

#define CLA0MUX0_TIMER0_CH1              CLA_MUX0(6)                        /*!< the input of CLA0MUX0 is TIMER0_CH2 */
#define CLA1MUX0_TIMER0_CH3              CLA_MUX0(6)                        /*!< the input of CLA0MUX0 is TIMER0_CH2 */
#define CLA2MUX0_TIMER0_CH3              CLA_MUX0(6)                        /*!< the input of CLA0MUX0 is TIMER0_CH2 */
#define CLA3MUX0_TIMER1_CH0              CLA_MUX0(6)                        /*!< the input of CLA0MUX0 is TIMER0_CH2 */

#define CLA0MUX0_TIMER0_CH2              CLA_MUX0(7)                        /*!< the input of CLA0MUX0 is TIMER0_CH2 */
#define CLA1MUX0_TIMER1_CH0              CLA_MUX0(7)                        /*!< the input of CLA1MUX0 is TIMER1_CH0 */
#define CLA2MUX0_TIMER1_CH1              CLA_MUX0(7)                        /*!< the input of CLA2MUX0 is TIMER1_CH1 */
#define CLA3MUX0_TIMER1_CH1              CLA_MUX0(7)                        /*!< the input of CLA3MUX0 is TIMER1_CH1 */

#define CLA0MUX0_CLAIN0                  CLA_MUX0(8)                        /*!< the input of CLA0MUX0 is CLAIN0(PA15) */
#define CLA1MUX0_CLAIN4                  CLA_MUX0(8)                        /*!< the input of CLA1MUX0 is CLAIN4(PB6) */
#define CLA2MUX0_CLAIN0                  CLA_MUX0(8)                        /*!< the input of CLA2MUX0 is CLAIN0(PA15) */
#define CLA3MUX0_CLAIN2                  CLA_MUX0(8)                        /*!< the input of CLA3MUX0 is CLAIN2(PB4) */

#define CLA0MUX0_CLAIN2                  CLA_MUX0(9)                        /*!< the input of CLA0MUX0 is CLAIN2(PB4) */
#define CLA1MUX0_CLAIN5                  CLA_MUX0(9)                        /*!< the input of CLA1MUX0 is CLAIN5(PB7) */
#define CLA2MUX0_CLAIN1                  CLA_MUX0(9)                        /*!< the input of CLA2MUX0 is CLAIN1(PB3) */
#define CLA3MUX0_CLAIN3                  CLA_MUX0(9)                        /*!< the input of CLA3MUX0 is CLAIN3(PB5) */

#define CLA0MUX0_CLAIN4                  CLA_MUX0(10)                       /*!< the input of CLA0MUX0 is CLAIN4(PB6) */
#define CLA1MUX0_CLAIN8                  CLA_MUX0(10)                       /*!< the input of CLA1MUX0 is CLAIN8(PB0) */
#define CLA2MUX0_CLAIN8                  CLA_MUX0(10)                       /*!< the input of CLA2MUX0 is CLAIN8(PB0) */
#define CLA3MUX0_CLAIN6                  CLA_MUX0(10)                       /*!< the input of CLA3MUX0 is CLAIN6(PB8) */

#define CLA0MUX0_CLAIN6                  CLA_MUX0(11)                       /*!< the input of CLA0MUX0 is CLAIN6(PB8) */
#define CLA1MUX0_CLAIN10                 CLA_MUX0(11)                       /*!< the input of CLA1MUX0 is CLAIN10(PB2) */
#define CLA2MUX0_CLAIN9                  CLA_MUX0(11)                       /*!< the input of CLA2MUX0 is CLAIN9(PB1) */
#define CLA3MUX0_CLAIN7                  CLA_MUX0(11)                       /*!< the input of CLA3MUX0 is CLAIN7(PB9) */

#define CLA0MUX0_CLAIN8                  CLA_MUX0(12)                       /*!< the input of CLA0MUX0 is CLAIN8(PB0) */
#define CLA1MUX0_CLAIN12                 CLA_MUX0(12)                       /*!< the input of CLA1MUX0 is CLAIN12(PA9) */
#define CLA2MUX0_CLAIN14                 CLA_MUX0(12)                       /*!< the input of CLA2MUX0 is CLAIN14(PA11) */
#define CLA3MUX0_CLAIN10                 CLA_MUX0(12)                       /*!< the input of CLA3MUX0 is CLAIN10(PB2) */

#define CLA0MUX0_CLAIN10                 CLA_MUX0(13)                       /*!< the input of CLA0MUX0 is CLAIN10(PB2) */
#define CLA1MUX0_CLAIN13                 CLA_MUX0(13)                       /*!< the input of CLA1MUX0 is CLAIN13(PA10) */
#define CLA2MUX0_CLAIN15                 CLA_MUX0(13)                       /*!< the input of CLA2MUX0 is CLAIN15(PA12) */
#define CLA3MUX0_CLAIN11                 CLA_MUX0(13)                       /*!< the input of CLA3MUX0 is CLAIN11(PA8) */

#define CLA0MUX0_CLAIN12                 CLA_MUX0(14)                       /*!< the input of CLA0MUX0 is CLAIN12(PA9) */
#define CLA1MUX0_CLAIN16                 CLA_MUX0(14)                       /*!< the input of CLA1MUX0 is CLAIN16(PF0) */
#define CLA2MUX0_CLAIN16                 CLA_MUX0(14)                       /*!< the input of CLA2MUX0 is CLAIN16(PF0) */
#define CLA3MUX0_CLAIN18                 CLA_MUX0(14)                       /*!< the input of CLA3MUX0 is CLAIN18(PA0) */

#define CLA0MUX0_CLAIN14                 CLA_MUX0(15)                       /*!< the input of CLA0MUX0 is CLAIN14(PA11) */
#define CLA1MUX0_CLAIN18                 CLA_MUX0(15)                       /*!< the input of CLA1MUX0 is CLAIN18(PA0) */
#define CLA2MUX0_CLAIN17                 CLA_MUX0(15)                       /*!< the input of CLA2MUX0 is CLAIN17(PF1) */
#define CLA3MUX0_CLAIN19                 CLA_MUX0(15)                       /*!< the input of CLA3MUX0 is CLAIN19(PA1) */

/* multiplexer 1 input selection */
#define CLA_MUX1(regval)                 (BITS(0,3) & ((uint32_t)(regval) << 0))
#define CLA0MUX1_CLA0_ASYNC_OUT          CLA_MUX1(0)                        /*!< the input of CLA0MUX1 is CLA0_ASYNC_OUT */
#define CLA1MUX1_CLA0_ASYNC_OUT          CLA_MUX1(0)                        /*!< the input of CLA1MUX1 is CLA0_ASYNC_OUT */
#define CLA2MUX1_CLA0_ASYNC_OUT          CLA_MUX1(0)                        /*!< the input of CLA2MUX1 is CLA0_ASYNC_OUT */
#define CLA3MUX1_CLA0_ASYNC_OUT          CLA_MUX1(0)                        /*!< the input of CLA3MUX1 is CLA0_ASYNC_OUT */

#define CLA0MUX1_CLA1_ASYNC_OUT          CLA_MUX1(1)                        /*!< the input of CLA0MUX1 is CLA1_ASYNC_OUT */
#define CLA1MUX1_CLA1_ASYNC_OUT          CLA_MUX1(1)                        /*!< the input of CLA1MUX1 is CLA1_ASYNC_OUT */
#define CLA2MUX1_CLA1_ASYNC_OUT          CLA_MUX1(1)                        /*!< the input of CLA2MUX1 is CLA1_ASYNC_OUT */
#define CLA3MUX1_CLA1_ASYNC_OUT          CLA_MUX1(1)                        /*!< the input of CLA3MUX1 is CLA1_ASYNC_OUT */

#define CLA0MUX1_CLA2_ASYNC_OUT          CLA_MUX1(2)                        /*!< the input of CLA0MUX1 is CLA2_ASYNC_OUT */
#define CLA1MUX1_CLA2_ASYNC_OUT          CLA_MUX1(2)                        /*!< the input of CLA1MUX1 is CLA2_ASYNC_OUT */
#define CLA2MUX1_CLA2_ASYNC_OUT          CLA_MUX1(2)                        /*!< the input of CLA2MUX1 is CLA2_ASYNC_OUT */
#define CLA3MUX1_CLA2_ASYNC_OUT          CLA_MUX1(2)                        /*!< the input of CLA3MUX1 is CLA2_ASYNC_OUT */

#define CLA0MUX1_CLA3_ASYNC_OUT          CLA_MUX1(3)                        /*!< the input of CLA0MUX1 is CLA3_ASYNC_OUT */
#define CLA1MUX1_CLA3_ASYNC_OUT          CLA_MUX1(3)                        /*!< the input of CLA1MUX1 is CLA3_ASYNC_OUT */
#define CLA2MUX1_CLA3_ASYNC_OUT          CLA_MUX1(3)                        /*!< the input of CLA2MUX1 is CLA3_ASYNC_OUT */
#define CLA3MUX1_CLA3_ASYNC_OUT          CLA_MUX1(3)                        /*!< the input of CLA3MUX1 is CLA3_ASYNC_OUT */

#define CLA0MUX1_ADC_CONV                CLA_MUX1(4)                        /*!< the input of CLA0MUX1 is ADC_CONV */
#define CLA1MUX1_ADC_CONV                CLA_MUX1(4)                        /*!< the input of CLA1MUX1 is ADC_CONV */
#define CLA2MUX1_ADC_CONV                CLA_MUX1(4)                        /*!< the input of CLA2MUX1 is ADC_CONV */
#define CLA3MUX1_ADC_CONV                CLA_MUX1(4)                        /*!< the input of CLA3MUX1 is ADC_CONV */

#define CLA0MUX1_TIMER0_CH3              CLA_MUX1(5)                        /*!< the input of CLA0MUX1 is TIMER0_CH3 */
#define CLA1MUX1_TIMER0_CH1              CLA_MUX1(5)                        /*!< the input of CLA1MUX1 is TIMER0_CH1 */
#define CLA2MUX1_TIMER0_CH0              CLA_MUX1(5)                        /*!< the input of CLA2MUX1 is TIMER0_CH0 */
#define CLA3MUX1_TIMER0_CH0              CLA_MUX1(5)                        /*!< the input of CLA3MUX1 is TIMER0_CH0 */

#define CLA0MUX1_TIMER1_CH0              CLA_MUX1(6)                        /*!< the input of CLA0MUX1 is TIMER1_CH0 */
#define CLA1MUX1_TIMER0_CH2              CLA_MUX1(6)                        /*!< the input of CLA1MUX1 is TIMER0_CH2 */
#define CLA2MUX1_TIMER0_CH2              CLA_MUX1(6)                        /*!< the input of CLA2MUX1 is TIMER0_CH2 */
#define CLA3MUX1_TIMER0_CH1              CLA_MUX1(6)                        /*!< the input of CLA3MUX1 is TIMER0_CH1 */

#define CLA0MUX1_TIMER2_CH1              CLA_MUX1(7)                        /*!< the input of CLA0MUX1 is TIMER2_CH1 */
#define CLA1MUX1_TIMER2_CH1              CLA_MUX1(7)                        /*!< the input of CLA1MUX1 is TIMER2_CH1 */
#define CLA2MUX1_TIMER1_CH0              CLA_MUX1(7)                        /*!< the input of CLA2MUX1 is TIMER1_CH0 */
#define CLA3MUX1_TIMER0_CH3              CLA_MUX1(7)                        /*!< the input of CLA3MUX1 is TIMER0_CH3 */

#define CLA0MUX1_CLAIN1                  CLA_MUX1(8)                        /*!< the input of CLA0MUX1 is CLAIN1(PB3) */
#define CLA1MUX1_CLAIN6                  CLA_MUX1(8)                        /*!< the input of CLA1MUX1 is CLAIN6(PB8) */
#define CLA2MUX1_CLAIN2                  CLA_MUX1(8)                        /*!< the input of CLA2MUX1 is CLAIN2(PB4) */
#define CLA3MUX1_CLAIN0                  CLA_MUX1(8)                        /*!< the input of CLA3MUX1 is CLAIN0(PA15) */

#define CLA0MUX1_CLAIN3                  CLA_MUX1(9)                        /*!< the input of CLA0MUX1 is CLAIN3(PB5) */
#define CLA1MUX1_CLAIN7                  CLA_MUX1(9)                        /*!< the input of CLA1MUX1 is CLAIN7(PB9) */
#define CLA2MUX1_CLAIN3                  CLA_MUX1(9)                        /*!< the input of CLA2MUX1 is CLAIN3(PB5) */
#define CLA3MUX1_CLAIN1                  CLA_MUX1(9)                        /*!< the input of CLA3MUX1 is CLAIN1(PB3) */

#define CLA0MUX1_CLAIN5                  CLA_MUX1(10)                       /*!< the input of CLA0MUX1 is CLAIN5(PB7) */
#define CLA1MUX1_CLAIN9                  CLA_MUX1(10)                       /*!< the input of CLA1MUX1 is CLAIN9(PB1) */
#define CLA2MUX1_CLAIN10                 CLA_MUX1(10)                       /*!< the input of CLA2MUX1 is CLAIN10(PB2) */
#define CLA3MUX1_CLAIN4                  CLA_MUX1(10)                       /*!< the input of CLA3MUX1 is CLAIN4(PB6) */

#define CLA0MUX1_CLAIN6                  CLA_MUX1(11)                       /*!< the input of CLA0MUX1 is CLAIN6(PB8) */
#define CLA1MUX1_CLAIN11                 CLA_MUX1(11)                       /*!< the input of CLA1MUX1 is CLAIN11(PA8) */
#define CLA2MUX1_CLAIN11                 CLA_MUX1(11)                       /*!< the input of CLA2MUX1 is CLAIN11(PA8) */
#define CLA3MUX1_CLAIN5                  CLA_MUX1(11)                       /*!< the input of CLA3MUX1 is CLAIN5(PB7) */

#define CLA0MUX1_CLAIN9                  CLA_MUX1(12)                       /*!< the input of CLA0MUX1 is CLAIN9(PB1) */
#define CLA1MUX1_CLAIN14                 CLA_MUX1(12)                       /*!< the input of CLA1MUX1 is CLAIN14(PA11) */
#define CLA2MUX1_CLAIN12                 CLA_MUX1(12)                       /*!< the input of CLA2MUX1 is CLAIN12(PA9) */
#define CLA3MUX1_CLAIN8                  CLA_MUX1(12)                       /*!< the input of CLA3MUX1 is CLAIN8(PB0) */
 
#define CLA0MUX1_CLAIN11                 CLA_MUX1(13)                       /*!< the input of CLA0MUX1 is CLAIN11(PA8) */
#define CLA1MUX1_CLAIN15                 CLA_MUX1(13)                       /*!< the input of CLA1MUX1 is CLAIN15(PA12) */
#define CLA2MUX1_CLAIN13                 CLA_MUX1(13)                       /*!< the input of CLA2MUX1 is CLAIN13(PA10) */
#define CLA3MUX1_CLAIN9                  CLA_MUX1(13)                       /*!< the input of CLA3MUX1 is CLAIN9(PB1) */

#define CLA0MUX1_CLAIN13                 CLA_MUX1(14)                       /*!< the input of CLA0MUX1 is CLAIN13(PA10) */
#define CLA1MUX1_CLAIN17                 CLA_MUX1(14)                       /*!< the input of CLA1MUX1 is CLAIN17(PF1) */
#define CLA2MUX1_CLAIN18                 CLA_MUX1(14)                       /*!< the input of CLA2MUX1 is CLAIN18(PA0) */
#define CLA3MUX1_CLAIN16                 CLA_MUX1(14)                       /*!< the input of CLA3MUX1 is CLAIN16(PF0) */

#define CLA0MUX1_CLAIN15                 CLA_MUX1(15)                       /*!< the input of CLA0MUX1 is CLAIN15(PA12) */
#define CLA1MUX1_CLAIN19                 CLA_MUX1(15)                       /*!< the input of CLA1MUX1 is CLAIN19(PA1) */
#define CLA2MUX1_CLAIN19                 CLA_MUX1(15)                       /*!< the input of CLA2MUX1 is CLAIN19(PA1) */
#define CLA3MUX1_CLAIN17                 CLA_MUX1(15)                       /*!< the input of CLA3MUX1 is CLAIN17(PF1) */

/* LUT control */
#define LUTCTL(regval)                   (BITS(0,7) & ((uint32_t)(regval) << 0))

/* CLA output selection */
#define FLIP_FLOP_OUTPUT                 ((uint32_t )0x00000000)            /*!< flip-flop output is selected as CLAx output */
#define LUT_RESULT                       CLA_CTL_OSEL                       /*!< LUT result is selected as CLAx output */

/* flip-flop clock polarity selection */
#define CLA_CLOCKPOLARITY_POSEDGE        ((uint32_t)0x00000000U)            /*!< the clock polarity of flip-flop is posedge */
#define CLA_CLOCKPOLARITY_NEGEDGE        CLA_CTL_CPOL                       /*!< the clock polarity of flip-flop is negedge */

/* flip-flop clock source selection */
#define PRE_CLA_LUT_RESULT               ((uint32_t)0x00000000U)            /*!< the LUT result of the previous CLA units */
#define MUX0_OUTPUT                      ((uint32_t)0x00000001U)            /*!< the multiplexer output of MUX0 */
#define HCLK                             ((uint32_t)0x00000002U)            /*!< HCLK */
#define TIMER_TRGO                       ((uint32_t)0x00000003U)            /*!< TIMER_TRGO */

/* function declarations */
/* CLA initialization and configuration functions */
/* reset CLA */
void cla_deinit(void);
/* enable CLA */
void cla_enable(cla_enum cla_periph);
/* disable CLA */
void cla_disable(cla_enum cla_periph);  
/* get CLA output state */
cla_outputstatus_enum cla_output_state_get(cla_enum cla_periph);
/* configure multiplexer input */
void cla_multiplexer_input_config(cla_enum cla_periph, cla_mux_enum mux, uint32_t input);
/* configure CLA LUT control register value */
void cla_lut_control_config(cla_enum cla_periph, uint8_t lutctl_value);
/* configure CLA output */
void cla_output_config(cla_enum cla_periph, uint32_t output);
/* enable CLA output */
void cla_output_enable(cla_enum cla_periph);
/* disable CLA output */
void cla_output_disable(cla_enum cla_periph);

/* flip-flop configuration */
/* reset CLA flip-flop output */
void cla_flip_flop_output_reset(cla_enum cla_periph);
/* configure clock polarity of flip-flop */
void cla_flip_flop_clockpolarity_config(cla_enum cla_periph, uint32_t polarity);
/* configure clock source of flip-flop */
void cla_flip_flop_clocksource_config(cla_enum cla_periph, uint32_t clock_source);

/* flag and interrupt functions */
/* check CLA flag is set or not */
FlagStatus cla_flag_get(cla_flag_enum flag);
/*clear CLA flag */
void cla_flag_clear(cla_flag_enum flag);
/* enable CLA negedge interrupt */
void cla_negedge_interrupt_enable(uint32_t clanie);
/* disable CLA negedge interrupt */
void cla_negedge_interrupt_disable(uint32_t clanidis);
/* enable CLA posedge interrupt */
void cla_posedge_interrupt_enable(uint32_t clapie);
/* disable CLA posedge interrupt */
void cla_posedge_interrupt_disable(uint32_t clapidis);
/* check CLA interrupt flag is set or not */
FlagStatus cla_interrupt_flag_get(cla_interrupt_flag_enum int_flag);
/* clear CLA interrupt flag */
void cla_interrupt_flag_clear(cla_interrupt_flag_enum int_flag);

#endif /* GD32E232_CLA_H */
