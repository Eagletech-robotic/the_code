/*
 * timer.c
 *
 *  Created on: May 9, 2025
 *      Author: nboulay
 */

#include "iot01A/timer.h"
#include "main.h"
#include "utils/myprintf.hpp"

extern TIM_HandleTypeDef
    htim1; // configuré pour tourner librement avec un /30 sur 65535 pour mesurer un temps interne au step.

float scaler; // précalcul pour gagner du temps
void timer_reset() {
    HAL_TIM_Base_Start(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    uint32_t prescaler = htim1.Instance->PSC + 1;
    float freq = 120000000.0f;
    scaler= (1.0f * prescaler / freq) * 1000000.0f;
}

float timer_get_us() {
//    uint32_t prescaler = htim1.Instance->PSC + 1;
//    float freq = 120000000.0f;
    uint32_t ticks_ = __HAL_TIM_GET_COUNTER(&htim1);
    //return (1.0f * prescaler / freq) * ticks_ * 1000000.0f;
    return scaler * ticks_ ;
}
