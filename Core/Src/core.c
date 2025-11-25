/*
 * core.c
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#include "core.h"

void core(void *argument) {
	hw_init();

	double phi = 0;
	while (1) {
		double fac = (double)hw_kb_get(0) / 512.0;
		hw_led_set(
			(uint8_t)((sin(phi - PI * 0.0 / 3.0) + 1.0) * 100.0 * fac),
			(uint8_t)((sin(phi - PI * 2.0 / 3.0) + 1.0) * 100.0 * fac),
			(uint8_t)((sin(phi - PI * 4.0 / 3.0) + 1.0) * 100.0 * fac)
		);
		phi = fmod(phi + 0.001, 2 * PI);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
