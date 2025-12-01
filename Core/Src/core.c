/*
 * core.c
 *
 *  Created on: Nov 25, 2025
 *	  Author: mirco
 */

#include "core.h"

#define KB_EXIT(kb) (HW_KB_BTN_BACK(kb.pressed) || HW_KB_BTN_BIG(kb.pressed, 0, 3))
#define KB_UP(kb_event) (HW_KB_BTN_UP(kb_event) || HW_KB_BTN_BIG(kb_event, 1, 3))
#define KB_MID(kb_event) (HW_KB_BTN_OK(kb_event) || HW_KB_BTN_BIG(kb_event, 2, 3))
#define KB_DOWN(kb_event) (HW_KB_BTN_DOWN(kb_event) || HW_KB_BTN_BIG(kb_event, 3, 3))

uint8_t kb_any(hw_kb_buttons_t btns) {
	for (uint8_t i = 0; i < HW_KB_COLS; i++) {
		if (btns.column[i] != 0) {
			return 1;
		}
	}

	return 0;
}

uint8_t check_logo_skip() {
	hw_kb_update_t kb = hw_kb_get_update();
	return kb_any(kb.pressed);
}

void draw_logo(uint8_t animate) {
	check_logo_skip();
	check_logo_skip();

	hw_screen_draw_string_hv_center(HW_SCREEN_W / 2, 15, HW_SCREEN_HEX(0x808080), "leck");

	if (animate) {
		vTaskDelay(pdMS_TO_TICKS(200));
		if (check_logo_skip()) {
			animate = 0;
		}
	}

	hw_screen_draw_string_x2_hv_center(HW_SCREEN_W / 2, 35, HW_SCREEN_HEX(0x000000), "mein");

	const char *title = "eierPhone 3==>";
	uint32_t title_i = 0;
	for (uint32_t x = 4; x < HW_SCREEN_W - 4; x++) {
		if (title[title_i] != '\0') {
			if (x >= HW_SCREEN_W / 2 - strlen(title) * 16 / 2 + title_i * 16 + 8) {
				hw_screen_freeze = 1;
				hw_screen_draw_char_x2(HW_SCREEN_W / 2 - strlen(title) * 16 / 2 + title_i * 16, 50, HW_SCREEN_HEX(0x000000), title[title_i]);
				hw_screen_freeze = 0;
				title_i++;
			}
		}
		hw_screen_fill_rect(x, HW_SCREEN_H - 10, 1, 6, HW_SCREEN_HEX(0x12B4E6));

		if (animate) {
			vTaskDelay(pdMS_TO_TICKS(8));
			if (check_logo_skip()) {
				animate = 0;
			}
		}
	}
	// https://mycolor.space/
	const uint16_t extra_colors[] = {
		HW_SCREEN_HEX(0x845EC2), // 0
		HW_SCREEN_HEX(0xD65DB1), // 1
		HW_SCREEN_HEX(0xFF6F91), // 2
		HW_SCREEN_HEX(0xFF9671), // 3
		HW_SCREEN_HEX(0xFFC75F), // 4
		HW_SCREEN_HEX(0xF9F871), // 5
	};
	for (uint32_t i = 0; i < sizeof(extra_colors) / sizeof(*extra_colors); i++) {
		hw_screen_freeze = 1;
		hw_screen_draw_string_x2(HW_SCREEN_W / 2 - strlen(title) * 16 / 2 + i, 50 + i, extra_colors[i], title);
		hw_screen_draw_string_x2(HW_SCREEN_W / 2 - strlen(title) * 16 / 2, 50, HW_SCREEN_HEX(0x000000), title);
		hw_screen_freeze = 0;

		if (animate) {
			vTaskDelay(pdMS_TO_TICKS(100));
			if (check_logo_skip()) {
				animate = 0;
			}
		}
	}
}

void draw_btn(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color_border, uint16_t color_fill, uint16_t color_text, const char *text) {
	hw_screen_fill_rect_hv_center(x, y, w, h, color_border);
	hw_screen_fill_rect_hv_center(x, y, w - 4, h - 4, color_fill);
	hw_screen_draw_string_hv_center(x, y, color_text, text);
}

#define COLOR_DES HW_SCREEN_HEX(0x414141)
#define COLOR_SEL HW_SCREEN_HEX(0x595959)
#define COLOR_DWN HW_SCREEN_HEX(0x0059A7)
#define COLOR_DES_B HW_SCREEN_HEX(0x212636)
#define COLOR_SEL_B HW_SCREEN_HEX(0x003977)
#define COLOR_DWN_B HW_SCREEN_HEX(0x385AAF)

uint8_t app = 0;

uint8_t menu_selected;
uint8_t menu_down;
const char *menu_btns[] = {
	"main",
	"kb_test",
	"mem_test",
	"pretty_colors",
	"settings",
};
uint8_t menu_btns_redraw[sizeof(menu_btns) / sizeof(*menu_btns)];

void app_menu_init() {
	draw_logo(0);

	menu_selected = 0;
	menu_down = 0;
	memset(menu_btns_redraw, 1, sizeof(menu_btns_redraw) / sizeof(*menu_btns_redraw));
}

void app_menu() {
	hw_kb_update_t kb = hw_kb_get_update();
	if (KB_UP(kb.pressed)) {
		if (menu_selected > 0) {
			menu_btns_redraw[menu_selected] = 1;
			menu_selected--;
			menu_btns_redraw[menu_selected] = 1;
		}
	}
	if (KB_DOWN(kb.pressed)) {
		if (menu_selected < sizeof(menu_btns) / sizeof(*menu_btns) - 1) {
			menu_btns_redraw[menu_selected] = 1;
			menu_selected++;
			menu_btns_redraw[menu_selected] = 1;
		}
	}
	if (KB_MID(kb.pressed)) {
		menu_down = 1;
		menu_btns_redraw[menu_selected] = 1;
	}
	if (KB_MID(kb.released)) {
		menu_down = 0;
		menu_btns_redraw[menu_selected] = 1;
		app = menu_selected + 1;
		return;
	}

	for (uint32_t i = 0; i < sizeof(menu_btns) / sizeof(*menu_btns); i++) {
		if (menu_btns_redraw[i]) {
			menu_btns_redraw[i] = 0;
			uint16_t color_border = i == menu_selected ? (menu_down ? COLOR_DWN_B : COLOR_SEL_B) : COLOR_DES_B;
			uint16_t color_fill   = i == menu_selected ? (menu_down ? COLOR_DWN   : COLOR_SEL  ) : COLOR_DES;
			uint16_t color_text   = HW_SCREEN_HEX(0x000000);
			draw_btn(
				HW_SCREEN_W / 2, 100 + i * 34,
				150, 30,
				color_border, color_fill, color_text,
				menu_btns[i]
			);
		}
	}
}

void app_main_init() {
}

void app_main() {
	hw_kb_update_t kb = hw_kb_get_update();

	if (KB_EXIT(kb)) {
		app = 0;
		return;
	}

	hw_screen_draw_string_x2_hv_center(HW_SCREEN_W / 2, HW_SCREEN_H / 2, HW_SCREEN_HEX(0xFF0000), "mainz");
}

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

void app_kb_test_init() {
	draw_logo(0);
}

void app_kb_test() {
	hw_kb_update_t kb = hw_kb_get_update();

	if (KB_EXIT(kb)) {
		app = 0;
		return;
	}

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
}

#define MEM_TEST_HOLD_STEP_DELAY (250)
#define MEM_TEST_HOLD_STEP_TIME (100)
#define MEM_TEST_TILE_SIZE (5)
#define MEM_TEST_STEP (HW_SCREEN_W / MEM_TEST_TILE_SIZE * 3 * 16)

uint32_t mem_test_base_addr;
uint8_t mem_test_pressed_dir;
uint32_t mem_test_pressed_ticks;
uint32_t mem_test_held_counter;

void draw_mem_test() {
	static uint8_t data[3 * (HW_SCREEN_W / MEM_TEST_TILE_SIZE) * (HW_SCREEN_H / MEM_TEST_TILE_SIZE)];
	hw_flash_read(mem_test_base_addr, data, sizeof(data));
	uint32_t index = 0;
	for (uint16_t y = 0; y < HW_SCREEN_H / MEM_TEST_TILE_SIZE; y++) {
		for (uint16_t x = 0; x < HW_SCREEN_W / MEM_TEST_TILE_SIZE; x++) {
			hw_screen_fill_rect(x * MEM_TEST_TILE_SIZE, y * MEM_TEST_TILE_SIZE, MEM_TEST_TILE_SIZE, MEM_TEST_TILE_SIZE,
				HW_SCREEN_RGB(data[index + 0], data[index + 1], data[index + 2]));
			index += 3;
		}
	}
	hw_screen_fill_rect_hv_center(HW_SCREEN_W / 2, 10, 80, 12, HW_SCREEN_HEX(0xFFFFFF));
	hw_screen_draw_uint32_hex_hv_center(HW_SCREEN_W / 2, 10, HW_SCREEN_HEX(0x000000), mem_test_base_addr);
}

void app_mem_test_init() {
	mem_test_base_addr = 0;
	mem_test_pressed_dir = 0;
	mem_test_pressed_ticks = 0;
	mem_test_held_counter = 0;
	draw_mem_test();
}

void app_mem_test() {
	hw_kb_update_t kb = hw_kb_get_update();

	if (KB_EXIT(kb)) {
		app = 0;
		return;
	}

	uint8_t go_up = 0;
	uint8_t go_down = 0;

	if (KB_UP(kb.pressed)) {
		mem_test_pressed_dir = 0;
		mem_test_pressed_ticks = xTaskGetTickCount();
		mem_test_held_counter = 0;
		go_up = 1;
	}
	if (KB_DOWN(kb.pressed)) {
		mem_test_pressed_dir = 1;
		mem_test_pressed_ticks = xTaskGetTickCount();
		mem_test_held_counter = 0;
		go_down = 1;
	}

	if (mem_test_pressed_dir == 0 && KB_UP(kb.held)) {
		if (xTaskGetTickCount() > mem_test_pressed_ticks + MEM_TEST_HOLD_STEP_DELAY) {
			while (mem_test_held_counter < (xTaskGetTickCount() - mem_test_pressed_ticks - MEM_TEST_HOLD_STEP_DELAY) / MEM_TEST_HOLD_STEP_TIME) {
				go_up++;
				mem_test_held_counter++;
			}
		}
	}
	if (mem_test_pressed_dir == 1 && KB_DOWN(kb.held)) {
		if (xTaskGetTickCount() > mem_test_pressed_ticks + MEM_TEST_HOLD_STEP_DELAY) {
			while (mem_test_held_counter < (xTaskGetTickCount() - mem_test_pressed_ticks - MEM_TEST_HOLD_STEP_DELAY) / MEM_TEST_HOLD_STEP_TIME) {
				go_down++;
				mem_test_held_counter++;
			}
		}
	}

	uint32_t step = MEM_TEST_STEP;
	if (KB_MID(kb.held)) {
		step *= 0x10;
	}
	while (go_up > 0) {
		if (mem_test_base_addr >= step) {
			mem_test_base_addr -= step;
			draw_mem_test();
		}
		go_up--;
	}
	while (go_down > 0) {
		if (mem_test_base_addr + ((HW_SCREEN_W / MEM_TEST_TILE_SIZE) * (HW_SCREEN_H / MEM_TEST_TILE_SIZE)) + step <= 0x800000) {
			mem_test_base_addr += step;
			draw_mem_test();
		}
		go_down--;
	}
}

uint8_t pretty_colors_v;
uint32_t pretty_colors_last_switch;

void draw_pretty_colors() {
	hw_screen_freeze = 1;
	memset(hw_screen_buffer, pretty_colors_v, HW_SCREEN_W * HW_SCREEN_H * sizeof(uint16_t));
	draw_logo(0);
	hw_screen_freeze = 0;
}

void app_pretty_colors_init() {
	pretty_colors_v = 0xF0;
	draw_pretty_colors();
	pretty_colors_last_switch = xTaskGetTickCount();
}

void app_pretty_colors() {
	hw_kb_update_t kb = hw_kb_get_update();

	if (KB_EXIT(kb)) {
		app = 0;
		return;
	}

	if (xTaskGetTickCount() - pretty_colors_last_switch >= 666) {
		pretty_colors_v ^= 0xFF;
		draw_pretty_colors();
		pretty_colors_last_switch = xTaskGetTickCount();
	}
}

void core(void *argument) {
	hw_init();
	hw_screen_brightness(9);
	hw_led_set_hex(0x12B4E6);

	// fade white
	vTaskDelay(pdMS_TO_TICKS(50));
	for (uint32_t i = 0; i < 255; i += 25) {
		hw_screen_freeze = 1;
		hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, HW_SCREEN_RGB(i, i, i));
		hw_screen_freeze = 0;
		vTaskDelay(pdMS_TO_TICKS(25));
	}

	// logo animation
	hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, COLOR_WHITE);
	draw_logo(1);
	vTaskDelay(pdMS_TO_TICKS(100));

	app = 0;
	uint8_t last_app = 0xFF;
	while (1) {
		// app switch
		if (app != last_app) {
			hw_screen_freeze = 1;
			hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, COLOR_WHITE);
			switch (app) {
			case 1:
				app_main_init();
				break;
			case 2:
				app_kb_test_init();
				break;
			case 3:
				app_mem_test_init();
				break;
			case 4:
				app_pretty_colors_init();
				break;
			default:
				app_menu_init();
				break;
			}
		}
		last_app = app;

		// app loop
		switch (app) {
		case 1:
			app_main();
			break;
		case 2:
			app_kb_test();
			break;
		case 3:
			app_mem_test();
			break;
		case 4:
			app_pretty_colors();
			break;
		default: {
			if (app != 0) {
				hw_screen_fill_rect(0, 0, 5, 5, HW_SCREEN_HEX(0xFF0000));
			}
			app_menu();
			break;
		}
		}

		hw_screen_freeze = 0;
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
