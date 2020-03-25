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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_MCU_H
#define __HAL_MCU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define __enter_critical()           __disable_irq()
#define __exit_critical()            __enable_irq()
#define __reset()                    NVIC_SystemReset()
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_mcu_init(void);
void hal_mcu_wdg_start(void);
void hal_mcu_wdg_reset(void);
#endif /* __HAL_MCU_H */

/****** (C) COPYRIGHT 2020 Single-Thread Development Team. *****END OF FILE****/

