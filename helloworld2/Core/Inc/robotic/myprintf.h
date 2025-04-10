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
        counter_##__LINE__++;                \
        if (counter_##__LINE__ % 250 == 0) { \
            printf(fmt, ##__VA_ARGS__);      \
        }                                    \
    } while(0)
