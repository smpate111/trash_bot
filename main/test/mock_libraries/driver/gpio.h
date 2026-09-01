/*
    This file mimics the actual gpio.h file by replacing the hardware logic
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


// Fake GPIO enums that are changed to integer types.
typedef int gpio_int_type_t;
typedef int gpio_mode_t;
typedef int gpio_num_t;
typedef int gpio_pulldown_t;
typedef int gpio_pullup_t;
typedef void (*gpio_isr_t)(void*);


// Fake ESP error macros.
#define ESP_OK 0


// Fake GPIO macros that simulate hardware configurations.
#define GPIO_INTR_ANYEDGE     3
#define GPIO_INTR_DISABLE     0
#define GPIO_INTR_NEGEDGE     2
#define GPIO_MODE_INPUT       1
#define GPIO_MODE_OUTPUT      2
#define GPIO_NUM_1            1
#define GPIO_NUM_2            2
#define GPIO_NUM_4            4
#define GPIO_NUM_6            6
#define GPIO_NUM_7            7
#define GPIO_NUM_11           11
#define GPIO_NUM_12           12
#define GPIO_NUM_13           13
#define GPIO_NUM_14           14
#define GPIO_NUM_15           15
#define GPIO_NUM_16           16
#define GPIO_NUM_42           42
#define GPIO_PULLUP_DISABLE   0
#define GPIO_PULLUP_ENABLE    1
#define GPIO_PULLDOWN_DISABLE 0


// Create a fake struct for the GPIO configuration.
typedef struct {
    uint64_t pin_bit_mask;
    gpio_mode_t mode;
    gpio_pullup_t pull_up_en;
    gpio_pulldown_t pull_down_en;
    gpio_int_type_t intr_type;
} gpio_config_t;


// Declare the fake hardware functions.
esp_err_t gpio_reset_pin(gpio_num_t gpio_num);
esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig);
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
int gpio_get_level(gpio_num_t gpio_num);
esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void* args);


#ifdef __cplusplus
}
#endif