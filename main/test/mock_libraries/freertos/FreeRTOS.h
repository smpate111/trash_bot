/*
    This file mimics the actual FreeRTOS.h file by replacing the hardware logic
    with fake logic to simulate how the hardware would function.
*/

#ifndef MOCK_FREERTOS_H_
#define MOCK_FREERTOS_H_

/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <fff.h>
#include <stdint.h>

//  ============================================================


// Fake FreeRTOS types that are changed to void, integer, long, or pointer types.
typedef void* QueueHandle_t;
typedef uint32_t TickType_t;
typedef long BaseType_t;
typedef uint32_t UBaseType_t;
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);


// Fake FreeRTOS macros that simulate hardware configurations.
#define pdTRUE                1
#define pdFALSE               0
#define portMAX_DELAY         0xFFFFFFFF
#define portTICK_PERIOD_MS    1
#define pdMS_TO_TICKS(ms)     ((TickType_t)(ms))


#endif