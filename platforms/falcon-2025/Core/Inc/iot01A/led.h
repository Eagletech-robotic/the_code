#pragma once
/*
 * led.h
 *
 *  Created on: Sep 22, 2024
 *      Author: nboulay
 */
#ifdef __cplusplus
extern "C" {
#endif

void led_init();
void led_1(float r); // 0 < r < 1
#ifdef __cplusplus
}
#endif
