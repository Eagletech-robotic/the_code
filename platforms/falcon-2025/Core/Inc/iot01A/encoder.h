#pragma once
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif

extern void encoder_init(TIM_HandleTypeDef *htim);
uint32_t encoder_get_value(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif
