/*
 * core.c
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#include "core.h"

void draw_key(uint16_t x, uint16_t y, uint8_t pressed, uint8_t held, uint8_t released) {
	if (pressed) {
		hw_screen_fill_rect(x, y, 16, 16, COLOR_GREEN);
	} else if (released) {
		hw_screen_fill_rect(x, y, 16, 16, COLOR_RED);
	} else {
		hw_screen_fill_rect(x, y, 16, 16, COLOR_DGREY);
	}
	hw_screen_fill_rect(x + 4, y + 4, 8, 8, held ? COLOR_YELLOW : COLOR_DGREY);
}

void core(void *argument) {
	hw_init();

	hw_led_set(18, 180, 230);

	hw_screen_brightness(10);
    hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, COLOR_WHITE);

    hw_screen_draw_string_hv_center(120, 20, COLOR_DGREEN, "leck");
    hw_screen_draw_string_hv_center(120, 30, COLOR_DGREEN, "mein");
    hw_screen_draw_string_hv_center(120, 40, COLOR_DGREEN, "eierPhone 3==>");

	while (1) {
		hw_kb_update_t kb = hw_kb_get_update();

		draw_key(53, 85,
			HW_KB_BTN_UP(kb.pressed), HW_KB_BTN_UP(kb.held), HW_KB_BTN_UP(kb.released));
		draw_key(53, 110,
			HW_KB_BTN_DOWN(kb.pressed), HW_KB_BTN_DOWN(kb.held), HW_KB_BTN_DOWN(kb.released));
		draw_key(33, 97,
			HW_KB_BTN_LEFT(kb.pressed), HW_KB_BTN_LEFT(kb.held), HW_KB_BTN_LEFT(kb.released));
		draw_key(73, 97,
			HW_KB_BTN_RIGHT(kb.pressed), HW_KB_BTN_RIGHT(kb.held), HW_KB_BTN_RIGHT(kb.released));

		draw_key(113, 87,
			HW_KB_BTN_HOME(kb.pressed), HW_KB_BTN_HOME(kb.held), HW_KB_BTN_HOME(kb.released));
		draw_key(113, 107,
			HW_KB_BTN_POWER(kb.pressed), HW_KB_BTN_POWER(kb.held), HW_KB_BTN_POWER(kb.released));

		draw_key(153, 87,
			HW_KB_BTN_OK(kb.pressed), HW_KB_BTN_OK(kb.held), HW_KB_BTN_OK(kb.released));
		draw_key(173, 87,
			HW_KB_BTN_BACK(kb.pressed), HW_KB_BTN_BACK(kb.held), HW_KB_BTN_BACK(kb.released));

		for (uint8_t c = 0; c < 6; c++) {
			for (uint8_t r = 0; r < 3; r++) {
				draw_key(62 + c * 20, 140 + r * 20,
					HW_KB_BTN_SMALL(kb.pressed, c, r), HW_KB_BTN_SMALL(kb.held, c, r), HW_KB_BTN_SMALL(kb.released, c, r));
			}
		}
		for (uint8_t c = 0; c < 5; c++) {
			for (uint8_t r = 0; r < 4; r++) {
				draw_key(72 + c * 20, 210 + r * 20,
					HW_KB_BTN_BIG(kb.pressed, c, r), HW_KB_BTN_BIG(kb.held, c, r), HW_KB_BTN_BIG(kb.released, c, r));
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
