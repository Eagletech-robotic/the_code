#pragma once
/*
 * top.h
 *
 *  Created on: Sep 28, 2024
 *      Author: nboulay
 */

#ifdef __cplusplus
extern "C" {
#endif

void top_init_driver();
void top_in_loop();
int top_is_time_to_start();

#ifdef __cplusplus
}
#endif
