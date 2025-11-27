/*
 * hw.h
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#ifndef INC_HW_H_
#define INC_HW_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include <math.h>
#include <memory.h>
#include <string.h>

#include "main.h"
#include "defines.h"
#include "st7789v_reg.h"

#define HW_KB_COLS 6
#define HW_KB_ROWS 9

#define HW_KB_DEF_BTN(x, c, r) (((x).column[(c)] >> (r)) & 1)
#define HW_KB_BTN_UP(x)    HW_KB_DEF_BTN(x, 1, 1)
#define HW_KB_BTN_LEFT(x)  HW_KB_DEF_BTN(x, 0, 1)
#define HW_KB_BTN_RIGHT(x) HW_KB_DEF_BTN(x, 3, 1)
#define HW_KB_BTN_DOWN(x)  HW_KB_DEF_BTN(x, 2, 1)
#define HW_KB_BTN_HOME(x)  HW_KB_DEF_BTN(x, 0, 0)
#define HW_KB_BTN_POWER(x) HW_KB_DEF_BTN(x, 2, 0)
#define HW_KB_BTN_OK(x)    HW_KB_DEF_BTN(x, 4, 1)
#define HW_KB_BTN_BACK(x)  HW_KB_DEF_BTN(x, 5, 1)
#define HW_KB_BTN_SMALL(x, c_btn, r_btn) HW_KB_DEF_BTN(x, c_btn, 2 + r_btn) // 6x3
#define HW_KB_BTN_BIG(x, c_btn, r_btn)   HW_KB_DEF_BTN(x, c_btn, 5 + r_btn) // 5x4

typedef struct hw_kb_buttons {
	uint16_t column[HW_KB_COLS];
} hw_kb_buttons_t;

typedef struct hw_kb_update {
	hw_kb_buttons_t pressed;
	hw_kb_buttons_t held;
	hw_kb_buttons_t released;
} hw_kb_update_t;

#define HW_SCREEN_W 240
#define HW_SCREEN_H 320

void hw_init();
uint8_t hw_bat_charging();
float hw_bat_volts();
void hw_uart_tx(uint8_t *data, uint32_t len);
uint32_t hw_uart_rx(uint8_t *data, uint32_t len);
void hw_led_set(uint8_t r, uint8_t g, uint8_t b);
void hw_flash_read();
void hw_flash_write();
hw_kb_update_t hw_kb_get_update();
void hw_screen_led_timer_callback();
void hw_screen_brightness(int8_t level);
void hw_screen_set_pixel(uint16_t x, uint16_t y, uint16_t color);
void hw_screen_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
#define hw_screen_fill_rect_h_center(x, y, w, h, color) hw_screen_fill_rect((x) - (w) / 2, y, w, h, color)
#define hw_screen_fill_rect_v_center(x, y, w, h, color) hw_screen_fill_rect(x, (y) - (h) / 2, w, h, color)
#define hw_screen_fill_rect_hv_center(x, y, w, h, color) hw_screen_fill_rect((x) - (w) / 2, (y) - (h) / 2, w, h, color)
void hw_screen_draw_data(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data);
void hw_screen_draw_char(uint16_t x, uint16_t y, uint16_t color, char c);
void hw_screen_draw_string(uint16_t x, uint16_t y, uint16_t color, const char *s);
#define hw_screen_draw_string_h_center(x, y, color, s) hw_screen_draw_string((x) - strlen(s) * 8 / 2, y, color, s)
#define hw_screen_draw_string_v_center(x, y, color, s) hw_screen_draw_string(x, (y) - 4, color, s)
#define hw_screen_draw_string_hv_center(x, y, color, s) hw_screen_draw_string((x) - strlen(s) * 8 / 2, (y) - 4, color, s)

#endif /* INC_HW_H_ */
