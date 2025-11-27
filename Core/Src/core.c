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
	hw_screen_brightness(9);
	hw_led_set_hex(0x12B4E6);

	// fade white
    vTaskDelay(pdMS_TO_TICKS(50));
    for (uint32_t i = 0; i < 255; i += 10) {
        hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, HW_SCREEN_RGB(i, i, i));
    	vTaskDelay(pdMS_TO_TICKS(10));
    }

    // start animation
    hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, COLOR_WHITE);
    hw_screen_draw_string_hv_center(HW_SCREEN_W / 2, 15, HW_SCREEN_HEX(0x808080), "leck");
    hw_screen_draw_string_x2_hv_center(HW_SCREEN_W / 2, 35, HW_SCREEN_HEX(0x000000), "mein");
    const char *title = "eierPhone 3==>";
    uint32_t title_i = 0;
    for (uint32_t x = 4; x < HW_SCREEN_W - 4; x++) {
    	if (title[title_i] != '\0') {
			if (x >= HW_SCREEN_W / 2 - strlen(title) * 16 / 2 + title_i * 16 + 8) {
				hw_screen_draw_char_x2(HW_SCREEN_W / 2 - strlen(title) * 16 / 2 + title_i * 16, 50, HW_SCREEN_HEX(0x000000), title[title_i]);
				title_i++;
			}
    	}
    	hw_screen_fill_rect(x, HW_SCREEN_H - 10, 1, 6, HW_SCREEN_HEX(0x12B4E6));
		vTaskDelay(pdMS_TO_TICKS(15));
    }
    // https://mycolor.space/
    const uint16_t extra_colors[] = {
		HW_SCREEN_HEX(0xF9F871), // 5
		HW_SCREEN_HEX(0xFFC75F), // 4
		HW_SCREEN_HEX(0xFF9671), // 3
		HW_SCREEN_HEX(0xFF6F91), // 2
		HW_SCREEN_HEX(0xD65DB1), // 1
		HW_SCREEN_HEX(0x845EC2), // 0
    };
    for (uint32_t i = 0; i < sizeof(extra_colors) / sizeof(*extra_colors); i++) {
    	uint32_t offset = (sizeof(extra_colors) / sizeof(*extra_colors) - i - 1) * 1;
    	hw_screen_draw_string_x2(HW_SCREEN_W / 2 - strlen(title) * 16 / 2 + offset, 50 + offset, extra_colors[i], title);
    	hw_screen_draw_string_x2(HW_SCREEN_W / 2 - strlen(title) * 16 / 2, 50, HW_SCREEN_HEX(0x000000), title);
    	vTaskDelay(pdMS_TO_TICKS(100));
    }

    // keyboard test
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
