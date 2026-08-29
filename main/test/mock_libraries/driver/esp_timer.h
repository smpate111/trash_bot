/*
    This file mimics the actual esp_timer.h file by replacing the hardware logic
    with fake logic to simulate how the hardware would function.
*/

#pragma once

/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <stdint.h>

//  ============================================================

#ifdef __cplusplus
extern "C" {
#endif

// Fakes the ESP32 hardware timer.
int64_t esp_timer_get_time(void);

#ifdef __cplusplus
}
#endif