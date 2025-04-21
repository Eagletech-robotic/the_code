/*
 * encoder.h
 *
 *  Created on: Oct 4, 2024
 *      Author: nboulay
 */

#pragma once
#include "main.h"
void encoder_init(TIM_HandleTypeDef *htim);
uint32_t encoder_get_value(TIM_HandleTypeDef *htim);
