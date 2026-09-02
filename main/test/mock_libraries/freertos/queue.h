/*
    This file mimics the actual queue.h file by replacing the hardware logic
    with fake logic to simulate how the hardware would function.
*/

#ifndef MOCK_QUEUE_H_
#define MOCK_QUEUE_H_

/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <freertos/FreeRTOS.h>

//  ============================================================


// Declare fake queue functions to simulate FreeRTOS queue behavior.
DECLARE_FAKE_VALUE_FUNC(QueueHandle_t, xQueueCreate, UBaseType_t, UBaseType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueSend, QueueHandle_t, const void*, TickType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueSendToFront, QueueHandle_t, const void*, TickType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueReceive, QueueHandle_t, void*, TickType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xQueueReset, QueueHandle_t);


#endif