/*
 * encoder.c
 *
 *  Created on: Oct 4, 2024
 *      Author: nboulay
 */


#include"encoder.h"

// les 2 channels doivent être configuré dans l'ioc.

void encoder_init(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

uint32_t encoder_get_value(TIM_HandleTypeDef *htim) {
	return htim->Instance->CNT;
}
