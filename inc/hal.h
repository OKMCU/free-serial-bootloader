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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_H
#define __HAL_H
/* Includes ------------------------------------------------------------------*/
#include "bl_config.h"
/* Exported constants --------------------------------------------------------*/
#define HAL_OK                  0
#define HAL_ERR                 -1
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void hal_board_init(void);
void hal_enter_critical(void);
void hal_exit_critical(void);
void hal_reset(void);
void hal_jump_to_addr(uint32_t addr);
void hal_mcu_get_uuid(char *p_buf, uint8_t size);

int8_t hal_wdg_config(uint16_t timeout);
int8_t hal_wdg_start(void);
int8_t hal_wdg_refresh(void);

int8_t hal_fmc_sector_info(uint32_t sector_id, uint32_t *sector_addr, uint32_t *sector_size);
int8_t hal_fmc_mass_erase(void);
int8_t hal_fmc_sector_erase(uint32_t sector_id);
int8_t hal_fmc_read(uint32_t addr, uint8_t *p_buf, uint32_t size);
int8_t hal_fmc_write(uint32_t addr, const uint8_t *p_buf, uint32_t size);

int8_t hal_eep_sector_info(uint32_t sector_id, uint32_t *sector_addr, uint32_t *sector_size);
int8_t hal_eep_mass_erase(void);
int8_t hal_eep_sector_erase(uint32_t sector_id);
int8_t hal_eep_read(uint32_t addr, uint8_t *p_buf, uint32_t size);
int8_t hal_eep_write(uint32_t addr, const uint8_t *p_buf, uint32_t size);

int8_t hal_uart_config(uint32_t baudrate, void (*p_fxn_on_recv_byte)(uint8_t rx_byte));
int8_t hal_uart_send(uint8_t tx_byte);

#endif /* __HAL_H */

/******************************** END OF FILE *********************************/

