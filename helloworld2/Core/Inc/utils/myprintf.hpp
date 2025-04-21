/*
 * myprintf.h
 *
 *  Created on: Apr 10, 2025
 *      Author: nboulay
 */

#pragma once
#include <stdio.h>

#ifdef MYPRINTF_ALWAYS
// Standard printf. For use on a PC.
#define myprintf(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
// Print every n messages. For use on MCU.
#define myprintf(fmt, ...)                                                                                             \
    do {                                                                                                               \
        static int counter_##__LINE__ = 0;                                                                             \
        if (counter_##__LINE__ % 250 == 0) {                                                                           \
            printf(fmt, ##__VA_ARGS__);                                                                                \
        }                                                                                                              \
        counter_##__LINE__++;                                                                                          \
    } while (0)
#endif
