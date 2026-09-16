/*
    This file mimics the actual esp_err.h file by replacing the hardware logic
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


// Fake ESP error enums that are changed to integer types.
typedef int esp_err_t;

// Fake ESP macros that simulate hardware configurations.
#define ESP_OK      0
#define ESP_FAIL    -1

// Declare the fake hardware functions.
const char *esp_err_to_name(esp_err_t code);

#ifdef __cplusplus
}
#endif