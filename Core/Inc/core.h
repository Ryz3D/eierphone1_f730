/*
 * core.h
 *
 *  Created on: Nov 25, 2025
 *      Author: mirco
 */

#ifndef INC_CORE_H_
#define INC_CORE_H_

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "defines.h"
#include "hw.h"

void core(void *argument);

#endif /* INC_CORE_H_ */
