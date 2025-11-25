/*
 * hw.c
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#include "hw.h"
#include "usbd_cdc_if.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern QSPI_HandleTypeDef hqspi;
extern SRAM_HandleTypeDef hsram1;

extern USBD_HandleTypeDef hUsbDeviceFS;

extern int __io_putchar(int ch) {
	if (ch != '\0') {
		/*
		CDC_Transmit_FS((uint8_t *)&ch, 1);
		USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
		for (uint64_t start = xTaskGetTickCount(); xTaskGetTickCount() < start + pdMS_TO_TICKS(1);) {
			if (hcdc->TxState == 0) {
				break;
			}
		}
		*/
		return ch;
	}
	return 0;
}

void hw_init() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

uint8_t hw_bat_chrg() {
	// TODO: gpio input
}

float hw_bat_volts() {
	// TODO: adc1 inp9
}

void hw_uart_tx(uint8_t *data, uint32_t len) {
	HAL_UART_Transmit(&huart6, data, len, 1000);
}

uint32_t hw_uart_rx(uint8_t *data, uint32_t len) {
	HAL_UART_Receive(&huart6, data, len, 1000);
}

void hw_led_set(uint8_t r, uint8_t g, uint8_t b) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, r);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, g);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, b);
}

uint32_t hw_kb_get(uint8_t col) {
	GPIOC->ODR |= 0b111111;
	GPIOC->ODR &= ~(1UL << col);
	vTaskDelay(pdMS_TO_TICKS(1));
	return ~GPIOA->IDR & 0b111111111;
}

void hw_flash_read() {
	// TODO
}

void hw_flash_write() {
	// TODO
}

void hw_screen_cmd() {
	// TODO: WARNING: many pins are still input just to be safe
}

void hw_screen_show() {
	// TODO
}
