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
#define myprintf(fmt, ...)                                                                                             \
    do {                                                                                                               \
        printf(fmt, ##__VA_ARGS__);                                                                                    \
        fflush(stdout);                                                                                                \
    } while (0)

// Print only on a PC, discard on MCU.
#define host_printf(fmt, ...)                                                                                          \
    do {                                                                                                               \
        printf(fmt, ##__VA_ARGS__);                                                                                    \
        fflush(stdout);                                                                                                \
    } while (0)

// Print only on MCU, discard on PC.
#define mcu_printf(fmt, ...)                                                                                           \
    do {                                                                                                               \
    } while (0)

#else

// Print every n messages. For use on MCU.
#define myprintf(fmt, ...)                                                                                             \
    do {                                                                                                               \
        static int counter_##__LINE__ = 0;                                                                             \
        if (counter_##__LINE__ % 150 == 0) {                                                                           \
            printf(fmt, ##__VA_ARGS__);                                                                                \
        }                                                                                                              \
        counter_##__LINE__++;                                                                                          \
    } while (0)

#define host_printf(fmt, ...)                                                                                          \
    do {                                                                                                               \
    } while (0)

#define mcu_printf(fmt, ...)                                                                                           \
    do {                                                                                                               \
        myprintf(fmt, ##__VA_ARGS__);                                                                                  \
        fflush(stdout);                                                                                                \
    } while (0)

#endif
