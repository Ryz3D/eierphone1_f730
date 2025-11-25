/*
 * hw.h
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#ifndef INC_HW_H_
#define INC_HW_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#include "defines.h"

void hw_init();
uint8_t hw_bat_chrg();
float hw_bat_volts();
void hw_uart_tx(uint8_t *data, uint32_t len);
uint32_t hw_uart_rx(uint8_t *data, uint32_t len);
void hw_led_set(uint8_t r, uint8_t g, uint8_t b);
uint32_t hw_kb_get(uint8_t col);
void hw_flash_read();
void hw_flash_write();
void hw_screen_cmd();
void hw_screen_show();

#endif /* INC_HW_H_ */
