/*
    This file mimics the actual task.h file by replacing the hardware logic
    with fake logic to simulate how the hardware would function.
*/

#ifndef MOCK_TASK_H_
#define MOCK_TASK_H_

/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <freertos/FreeRTOS.h>

//  ============================================================


// Declare fake task functions to simulate FreeRTOS task behavior.
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xTaskCreate, TaskFunction_t, const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*);
DECLARE_FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
DECLARE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);

#endif