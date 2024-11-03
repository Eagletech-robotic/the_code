#pragma once
/*
 * top.h
 *
 *  Created on: Sep 28, 2024
 *      Author: nboulay
 */

#include "iot01A/top_driver.h"

void top_init(config_t* config);
void top_init_driver();
void top_in_loop();
int top_is_time_to_start();
