/*
    This file fleshes out the Motor class logic defined in motor.hpp.
*/

/*
    ============================================================
    Include the Motor class header file to access the class's
    methods and variables.
    ============================================================
*/
#include <../include/motors/motor.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Motor class that initializes the motor
    with user-defined values and configures the timer and
    channels.
    ============================================================
*/
Motor::Motor(const Motor_Config &motor_setup) {
    // Store the user-defined motor configurations.
    config = motor_setup;

    // Configure the pins to be output pins.
    gpio_reset_pin(config.in1_pin);
    gpio_reset_pin(config.in2_pin);
    gpio_set_direction(config.in1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.in2_pin, GPIO_MODE_OUTPUT);

    
    // Configure the channel for the IN1 pin.
    ledc_channel_config_t channel_1_config = {};
    channel_1_config.gpio_num = config.in1_pin;         // Set the GPIO pin for the channel output.
    channel_1_config.speed_mode = LEDC_LOW_SPEED_MODE;  // Have the hardware timer operate in low-speed.
    channel_1_config.channel = config.channel_1;        // Set the LEDC channel.
    channel_1_config.timer_sel = LEDC_TIMER_0;          // Pick the first hardware timer index for this channel to use.
    channel_1_config.duty = 0;                          // Configure the initial duty cycle to 0.
    channel_1_config.hpoint = 0;                        // Set the signal to pull high at the start of each timer cycle.

    ledc_channel_config(&channel_1_config);


    // Configure the channel for the IN2 pin.
    ledc_channel_config_t channel_2_config = {};
    channel_2_config.gpio_num = config.in2_pin;         // Set the GPIO pin for the channel output.
    channel_2_config.speed_mode = LEDC_LOW_SPEED_MODE;  // Have the hardware timer operate in low-speed.
    channel_2_config.channel = config.channel_2;        // Set the LEDC channel.
    channel_2_config.timer_sel = LEDC_TIMER_0;          // Pick the first hardware timer index for this channel to use.
    channel_2_config.duty = 0;                          // Configure the initial duty cycle to 0.
    channel_2_config.hpoint = 0;                        // Set the signal to pull high at the start of each timer cycle.

    ledc_channel_config(&channel_2_config);

    ESP_LOGI(config.name.c_str(), "Initialized motor on pins %d and %d.", config.in1_pin, config.in2_pin);
}
//  ============================================================


/*
    ============================================================
    Adjust's the motor's speed between 0 to 255.
    ============================================================
*/
void Motor::set_speed(uint32_t speed) {
    Current_Speed = speed;
    ESP_LOGI(config.name.c_str(), "Adjusting motor speed to: %lu.", Current_Speed);
    return;
}
//  ============================================================


/*
    ============================================================
    Spins the motor clockwise by setting the IN1 pin to
    HIGH and the IN2 pin to LOW.
    ============================================================
*/
void Motor::spin_forward() {
    ESP_LOGI(config.name.c_str(), "Spinning motor forward at speed: %lu.", Current_Speed);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, Current_Speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, 0.0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);
    return;
}
//  ============================================================


/*
    ============================================================
    Spins the motor counter-clockwise by setting the IN1 pin
    to LOW and the IN2 pin to HIGH.
    ============================================================
*/
void Motor::spin_backward() {
    ESP_LOGI(config.name.c_str(), "Spinning motor backward at speed: %lu.", Current_Speed);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, 0.0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, Current_Speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);
    return;
}
//  ============================================================


/*
    ============================================================
    Stops the motor from spinning by setting both the
    IN1 and IN2 pins to LOW.
    ============================================================
*/
void Motor::stop() {
    ESP_LOGI(config.name.c_str(), "Stopping motor from spinning.");

    set_speed(0.0);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, 0.0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, 0.0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);

    return;
}
//  ============================================================