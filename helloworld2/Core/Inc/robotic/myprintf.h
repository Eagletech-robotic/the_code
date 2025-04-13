/*
 * myprintf.h
 *
 *  Created on: Apr 10, 2025
 *      Author: nboulay
 */

#pragma once
#include <stdio.h>
#define myprintf(fmt, ...)           \
    do {                                     \
        static int counter_##__LINE__ = 0;   \
        if (counter_##__LINE__ % 250 == 0) { \
            printf(fmt, ##__VA_ARGS__);      \
        }                                    \
		counter_##__LINE__++;                \
    } while(0)
