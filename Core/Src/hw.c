/*
 * hw.c
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#include "hw.h"
#include "font.h"

extern ADC_HandleTypeDef hadc1; // Battery voltage
extern UART_HandleTypeDef huart6; // Debug UART
extern TIM_HandleTypeDef htim3; // RGB LED PWM timer
extern TIM_HandleTypeDef htim6; // LCD backlight controller EN pulse generator
extern QSPI_HandleTypeDef hqspi; // FLASH QSPI
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1; // LCD framebuffer DMA stream

#define USB_CDC 0

#if USB_CDC
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int __io_putchar(int ch) {
	if (ch != '\0') {
		CDC_Transmit_FS((uint8_t *)&ch, 1);
		USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
		for (uint64_t start = xTaskGetTickCount(); xTaskGetTickCount() < start + pdMS_TO_TICKS(1);) {
			if (hcdc->TxState == 0) {
				break;
			}
		}
		return ch;
	}
	return 0;
}
#endif

void hw_kb_loop(void *argument);
void hw_screen_cmd(uint16_t cmd);
void hw_screen_dat(uint16_t dat);

void screentest();

void hw_init() {
	// # bat

	// # led
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	// # kb
	TaskHandle_t hw_kb_task;
	xTaskCreate(hw_kb_loop, "hw_kb", 128, NULL, osPriorityLow, &hw_kb_task);

	// # screen
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_POW_EN_GPIO_Port, LCD_POW_EN_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(5));

	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(10));

	HAL_GPIO_WritePin(LCD_EXTC_GPIO_Port, LCD_EXTC_Pin, GPIO_PIN_SET);

	hw_screen_cmd(ST7789V_SLPOUT);
	vTaskDelay(pdMS_TO_TICKS(5));

	hw_screen_cmd(ST7789V_MADCTL); // Memory data acccess control
	hw_screen_dat(0x00);

	hw_screen_cmd(ST7789V_PORCTRL); // Porch Setting
	hw_screen_dat(0x18);
	hw_screen_dat(0x18);
	hw_screen_dat(0x00);
	hw_screen_dat(0x33);
	hw_screen_dat(0x33);

	hw_screen_cmd(ST7789V_GCTRL); // Gate Control
	hw_screen_dat(0x70); // VGH, VGL

	hw_screen_cmd(ST7789V_VCOMS);
	hw_screen_dat(0x3A);

	hw_screen_cmd(ST7789V_LCMCTRL);
	hw_screen_dat(0x2C);

	hw_screen_cmd(ST7789V_VDVVRHEN);
	hw_screen_dat(0x01);

	hw_screen_cmd(ST7789V_VRHS);
	hw_screen_dat(0x14);

	hw_screen_cmd(ST7789V_VDVS);
	hw_screen_dat(0x20);

	hw_screen_cmd(ST7789V_PWCTRL1);
	hw_screen_dat(0xA4);
	hw_screen_dat(0xA1); // AVDD VCL

	hw_screen_cmd(ST7789V_PVGAMCTRL);
	hw_screen_dat(0xD0);
	hw_screen_dat(0x07);
	hw_screen_dat(0x0D);
	hw_screen_dat(0x09);
	hw_screen_dat(0x08);
	hw_screen_dat(0x25);
	hw_screen_dat(0x28);
	hw_screen_dat(0x53);
	hw_screen_dat(0x39);
	hw_screen_dat(0x12);
	hw_screen_dat(0x0B);
	hw_screen_dat(0x0A);
	hw_screen_dat(0x17);
	hw_screen_dat(0x34);

	hw_screen_cmd(ST7789V_NVGAMCTRL);
	hw_screen_dat(0xD0);
	hw_screen_dat(0x07);
	hw_screen_dat(0x0D);
	hw_screen_dat(0x09);
	hw_screen_dat(0x09);
	hw_screen_dat(0x25);
	hw_screen_dat(0x29);
	hw_screen_dat(0x35);
	hw_screen_dat(0x39);
	hw_screen_dat(0x13);
	hw_screen_dat(0x0A);
	hw_screen_dat(0x0A);
	hw_screen_dat(0x16);
	hw_screen_dat(0x34);

	hw_screen_cmd(ST7789V_FRCTRL2);
	hw_screen_dat(0x0F); // 60 FPS
	hw_screen_cmd(ST7789V_TEON);
	hw_screen_dat(0x00); // Tear effect pin enable
	hw_screen_cmd(ST7789V_COLMOD);
	hw_screen_dat(0x55); // 65K colors, 16 bit/px
	hw_screen_cmd(ST7789V_INVON);

	hw_screen_fill_rect(0, 0, HW_SCREEN_W, HW_SCREEN_H, COLOR_BLACK);
	hw_screen_cmd(ST7789V_DISPON);
	HAL_GPIO_WritePin(LCD_LIGHT_GPIO_Port, LCD_LIGHT_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(1));
}

uint8_t hw_bat_charging() {
	return HAL_GPIO_ReadPin(BAT_CHRG_GPIO_Port, BAT_CHRG_Pin) == GPIO_PIN_SET;
}

float hw_bat_volts() {
	// TODO: adc1 inp9
	return 4.2f;
}

void hw_uart_tx(uint8_t *data, uint32_t len) {
	HAL_UART_Transmit(&huart6, data, len, 1000);
}

uint32_t hw_uart_rx(uint8_t *data, uint32_t len) {
	uint32_t i;
	for (i = 0; i < len; i++) {
		if (HAL_UART_Receive(&huart6, data, 1, 10) != HAL_OK) {
			break;
		}
		data++;
	}
	return i;
}

void hw_led_set(uint8_t r, uint8_t g, uint8_t b) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, r);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, g);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, b);
}

void hw_flash_read() {
	// TODO
}

void hw_flash_write() {
	// TODO
}

volatile hw_kb_buttons_t hw_kb_last_held = { 0 };
volatile hw_kb_update_t hw_kb_update = { 0 };

hw_kb_update_t hw_kb_get_update() {
	hw_kb_update_t temp = hw_kb_update;
	memset((void*)&hw_kb_update, 0, sizeof(hw_kb_update_t));
	hw_kb_last_held = temp.held;
	return temp;
}

void hw_kb_loop(void *argument) {
	while (1) {
		for (uint8_t i = 0; i < HW_KB_COLS; i++) {
			GPIOC->ODR |= 0b111111;
			GPIOC->ODR &= ~(1UL << i);
			vTaskDelay(pdMS_TO_TICKS(1));

			hw_kb_update.held.column[i] = ~GPIOA->IDR & 0b111111111;
			hw_kb_update.pressed.column[i] |= hw_kb_update.held.column[i] & ~hw_kb_last_held.column[i];
			hw_kb_update.released.column[i] |= ~hw_kb_update.held.column[i] & (hw_kb_last_held.column[i] | hw_kb_update.pressed.column[i]);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

int8_t hw_screen_brightness_current = 15;
volatile uint8_t hw_screen_brightness_pulses = 0;

void hw_screen_led_timer_callback() {
	HAL_GPIO_TogglePin(LCD_LIGHT_GPIO_Port, LCD_LIGHT_Pin);
	if (--hw_screen_brightness_pulses == 0) {
		HAL_TIM_Base_Stop_IT(&htim6);
	}
}

void hw_screen_brightness(int8_t level) {
	HAL_TIM_Base_Stop_IT(&htim6);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	if (level < 0) {
		level = 0;
	}
	if (level > 15) {
		level = 15;
	}
	hw_screen_brightness_pulses = hw_screen_brightness_current - level;
	hw_screen_brightness_current = level;
	while (hw_screen_brightness_pulses < 0) {
		hw_screen_brightness_pulses += 16;
	}
	hw_screen_brightness_pulses *= 2;
	if (hw_screen_brightness_pulses == 0) {
		return;
	}
	HAL_TIM_Base_Start_IT(&htim6);
}

#define HW_SCREEN_CMD (volatile uint16_t*)(0x60000000UL)
#define HW_SCREEN_DAT (volatile uint16_t*)(0x60000000UL + (1 << (16 + 1))) // RS = A16

void hw_screen_cmd(uint16_t cmd) {
	*HW_SCREEN_CMD = cmd;
}

void hw_screen_dat(uint16_t dat) {
	*HW_SCREEN_DAT = dat;
}

void hw_screen_reg_w_16(uint8_t r, uint16_t d) {
	hw_screen_cmd(r);
	hw_screen_dat(d >> 8);
	hw_screen_dat(d);
}

void hw_screen_reg_w_32(uint8_t r, uint32_t d) {
	hw_screen_cmd(r);
	hw_screen_dat(d >> 24);
	hw_screen_dat(d >> 16);
	hw_screen_dat(d >> 8);
	hw_screen_dat(d);
}

void hw_screen_set_cursor(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	hw_screen_reg_w_32(ST7789V_CASET, ((uint32_t)x0 << 16) | x1);
	hw_screen_reg_w_32(ST7789V_RASET, ((uint32_t)y0 << 16) | y1);
    hw_screen_cmd(ST7789V_RAMWR);
}

static uint32_t hw_screen_buffer_aligned[HW_SCREEN_W * HW_SCREEN_H / 2];
uint16_t *hw_screen_buffer = (uint16_t*)hw_screen_buffer_aligned;
volatile uint8_t hw_screen_freeze = 0;

void hw_screen_stream_framebuffer() {
	if (!hw_screen_freeze && HAL_DMA_GetState(&hdma_memtomem_dma2_stream1) == HAL_DMA_STATE_READY) {
		hw_screen_set_cursor(0, 0, HW_SCREEN_W - 1, HW_SCREEN_H - 1);
		HAL_DMA_Start_IT(
			&hdma_memtomem_dma2_stream1,
			(uint32_t)(void*)hw_screen_buffer_aligned, // source
			(uint32_t)(void*)HW_SCREEN_DAT, // destination
			sizeof(hw_screen_buffer_aligned) / sizeof(*hw_screen_buffer_aligned) // length (in words)
		);
	}
}

void hw_screen_set_pixel(uint16_t x, uint16_t y, uint16_t color) {
	hw_screen_buffer[y * HW_SCREEN_W + x] = color;
}

void hw_screen_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	for (uint16_t x1 = x; x1 < x + w; x1++) {
		for (uint16_t y1 = y; y1 < y + h; y1++) {
			hw_screen_buffer[y1 * HW_SCREEN_W + x1] = color;
		}
	}
}

void hw_screen_draw_data(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data) {
	for (uint16_t y1 = y; y1 < y + h; y1++) {
		memcpy(&hw_screen_buffer[y1 * HW_SCREEN_W + x], data, w * sizeof(uint16_t));
		data += w;
	}
}

void hw_screen_draw_char(uint16_t x, uint16_t y, uint16_t color, char c) {
	if (c == ' ') {
		return;
	}
	uint32_t map_index = 66;
	for (uint32_t i = 0; font_charmap[i] != '\0'; i++) {
		if (font_charmap[i] == c) {
			map_index = i;
			break;
		}
	}
	for (uint8_t r = 0; r < 8; r++) {
		for (uint8_t c = 0; c < 8; c++) {
			if ((font[map_index * 8 + r] >> (7 - c)) & 1) {
				hw_screen_set_pixel(x + c, y + r, color);
			}
		}
	}
}

void hw_screen_draw_char_x2(uint16_t x, uint16_t y, uint16_t color, char c) {
	if (c == ' ') {
		return;
	}
	uint32_t map_index = 66;
	for (uint32_t i = 0; font_charmap[i] != '\0'; i++) {
		if (font_charmap[i] == c) {
			map_index = i;
			break;
		}
	}
	for (uint8_t r = 0; r < 8; r++) {
		for (uint8_t c = 0; c < 8; c++) {
			if ((font[map_index * 8 + r] >> (7 - c)) & 1) {
				hw_screen_fill_rect(x + c * 2, y + r * 2, 2, 2, color);
			}
		}
	}
}

void hw_screen_draw_string(uint16_t x, uint16_t y, uint16_t color, const char *s) {
	while (*s != '\0') {
		hw_screen_draw_char(x, y, color, *s++);
		x += 8;
	}
}

void hw_screen_draw_string_x2(uint16_t x, uint16_t y, uint16_t color, const char *s) {
	while (*s != '\0') {
		hw_screen_draw_char_x2(x, y, color, *s++);
		x += 16;
	}
}
