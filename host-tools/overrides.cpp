#include "./overrides.hpp"
#include <stdint.h>

#ifdef __EMSCRIPTEN__
// WASM Implementation
#include <emscripten/emscripten.h>

static double timer_start_ms = 0.0;

extern "C" {
    EMSCRIPTEN_KEEPALIVE void timer_reset() {
        timer_start_ms = emscripten_get_now();
    }

    EMSCRIPTEN_KEEPALIVE float timer_get_us() {
        return float((emscripten_get_now() - timer_start_ms) * 1000.0);
    }
}

#else
// X86 Implementation
#include <x86intrin.h> // For __rdtsc()
#include <time.h>

// Global variables
static uint64_t timer_start = 0;
static double ticks_per_us = 0.0;

// Initialize the timer system by estimating CPU frequency
static void timer_init() {
    if (ticks_per_us == 0.0) {
        struct timespec start_time, end_time;
        uint64_t start_tsc, end_tsc;

        // Get current time and TSC
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        start_tsc = __rdtsc();

        // Sleep for a short period to measure CPU frequency
        struct timespec sleep_time = {0, 50000000}; // 50ms for better accuracy
        nanosleep(&sleep_time, NULL);

        // Get new time and TSC
        end_tsc = __rdtsc();
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        // Calculate elapsed time in microseconds
        uint64_t elapsed_ns = (end_time.tv_sec - start_time.tv_sec) * 1000000000ULL +
                              (end_time.tv_nsec - start_time.tv_nsec);
        double elapsed_us = elapsed_ns / 1000.0;

        // Calculate ticks per microsecond
        ticks_per_us = (end_tsc - start_tsc) / elapsed_us;
    }
}

extern "C" {
void timer_reset() {
    // Initialize timer system if needed
    if (ticks_per_us == 0.0) {
        timer_init();
    }

    // Store current timestamp as start time
    timer_start = __rdtsc();
}

float timer_get_us() {
    // Initialize timer system if needed
    if (ticks_per_us == 0.0) {
        timer_init();
        timer_start = __rdtsc();
        return 0.0f;
    }

    // Read current timestamp and calculate elapsed microseconds
    uint64_t current = __rdtsc();
    return (float) ((current - timer_start) / ticks_per_us);
}
}
#endif
