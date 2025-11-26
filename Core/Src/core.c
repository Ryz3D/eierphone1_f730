/*
 * core.c
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#include "core.h"

void core(void *argument) {
	hw_init();

    hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, COLOR_WHITE);
    hw_screen_fill_rect(30,  150, 70,  35, COLOR_RED);
    hw_screen_fill_rect(110, 100, 110, 65, COLOR_BROWN);
    hw_screen_fill_rect(110, 175, 90,  40, COLOR_BLUE);

	double t = 0;
	while (1) {
		hw_kb_update_t kb = hw_kb_get_update();
		int16_t kb_col = -1;
		int16_t kb_row = -1;
		for (uint8_t c = 0; c < HW_KB_COLS; c++) {
			for (uint8_t r = 0; r < HW_KB_ROWS; r++) {
				if (kb.pressed.column[c] >> r) {
					kb_col = c;
					kb_row = c;
					break;
				}
			}
			if (kb_col != -1 && kb_row != -1) {
				break;
			}
		}
		if (kb_col != -1 && kb_row != -1) {
			double fac = (double)kb_row / HW_KB_ROWS;
			double hue = 2.0 * PI * (double)kb_col / HW_KB_COLS;
			hw_led_set(
				(uint8_t)((sin(hue - PI * 0.0 / 3.0) + 1.0) * 100.0 * fac),
				(uint8_t)((sin(hue - PI * 2.0 / 3.0) + 1.0) * 100.0 * fac),
				(uint8_t)((sin(hue - PI * 4.0 / 3.0) + 1.0) * 100.0 * fac)
			);
		}
		hw_screen_brightness((uint8_t)(-7.5 * cos(t) + 8.0));
		t += 0.01;
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
