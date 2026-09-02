/*
    This file defines the mock hardware GPIO and LEDC configurations
    used in multiple files.
*/

#pragma once

/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <driver/esp_timer.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <fff.h>
#include <unity.h>

//  ============================================================



// Declare the fake LEDC functions.
DECLARE_FAKE_VALUE_FUNC(esp_err_t, ledc_channel_config, const ledc_channel_config_t*);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, ledc_set_duty, ledc_mode_t, ledc_channel_t, uint32_t);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, ledc_update_duty, ledc_mode_t, ledc_channel_t);


// Declare the fake GPIO functions.
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpio_config, const gpio_config_t*);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpio_isr_handler_add, gpio_num_t, gpio_isr_t, void*);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpio_reset_pin, gpio_num_t);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpio_set_direction, gpio_num_t, gpio_mode_t);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpio_set_level, gpio_num_t, uint32_t);
DECLARE_FAKE_VALUE_FUNC(int, gpio_get_level, gpio_num_t);


// Declare the fake ESP32 timer.
DECLARE_FAKE_VALUE_FUNC(int64_t, esp_timer_get_time);