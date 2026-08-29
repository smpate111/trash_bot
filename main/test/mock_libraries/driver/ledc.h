/*
    This file mimics the actual ledc.h file by replacing the hardware logic
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


// Fake LEDC enums that are changed to integer types.
typedef int ledc_channel_t;
typedef int ledc_clk_cfg_t;
typedef int ledc_mode_t;
typedef int ledc_timer_t;
typedef int ledc_timer_bit_t;


// Fake LEDC macros that simulate hardware configurations.
#define LEDC_AUTO_CLK       0
#define LEDC_CHANNEL_0      0
#define LEDC_CHANNEL_1      1
#define LEDC_CHANNEL_2      2
#define LEDC_CHANNEL_3      3
#define LEDC_LOW_SPEED_MODE 0
#define LEDC_TIMER_0        0


// Create a fake struct for the LEDC timer configuration.
typedef struct {
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t  timer_num;
    uint32_t freq_hz;
    ledc_clk_cfg_t clk_cfg;
} ledc_timer_config_t;


// Create a fake struct for the LEDC channel configuration.
typedef struct {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_timer_t timer_sel;
    uint32_t duty;
    int hpoint;
    int intr_type;
} ledc_channel_config_t;


// Declare the fake hardware functions.
esp_err_t ledc_timer_config(const ledc_timer_config_t* timer_conf);
esp_err_t ledc_channel_config(const ledc_channel_config_t* ledc_conf);
esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel);


#ifdef __cplusplus
}
#endif