/*
    This file fleshes out the Motor class logic defined in motor.hpp.
*/

/*
    ============================================================
    Include the Motor class header file to access the class's
    methods and variables.
    ============================================================
*/
#include "motors/motor.hpp"
//  ============================================================



/*
    ============================================================
    Constructor for the Motor class that initializes the motor
    with user-defined values to configure the LEDC channels.
    ============================================================
*/
Motor::Motor(const Motor_Config &motor_setup) : config(motor_setup) {
    // Configure the pins to be output pins.
    esp_err_t err = gpio_reset_pin(config.in1_pin);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to reset GPIO Pin [%d].", config.in1_pin);
        return;
    }

    err = gpio_reset_pin(config.in2_pin);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to reset GPIO Pin [%d].", config.in2_pin);
        return;
    }

    err = gpio_set_direction(config.in1_pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure GPIO Pin [%d] as output.", config.in1_pin);
        return;
    }

    err = gpio_set_direction(config.in2_pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure GPIO Pin [%d] as output.", config.in2_pin);
        return;
    }

    
    // Configure the channel for the IN1 pin.
    ledc_channel_config_t channel_1_config = {};
    channel_1_config.gpio_num = config.in1_pin;         // Set the GPIO pin for the channel output.
    channel_1_config.speed_mode = LEDC_LOW_SPEED_MODE;  // Have the hardware timer operate in low-speed.
    channel_1_config.channel = config.channel_1;        // Set the LEDC channel.
    channel_1_config.timer_sel = LEDC_TIMER_0;          // Pick the first hardware timer index for this channel to use.
    channel_1_config.duty = 0;                          // Configure the initial duty cycle to 0.
    channel_1_config.hpoint = 0;                        // Set the signal to pull high at the start of each timer cycle.

    err = ledc_channel_config(&channel_1_config);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure motor LEDC channel [1].");
        return;
    }


    // Configure the channel for the IN2 pin.
    ledc_channel_config_t channel_2_config = {};
    channel_2_config.gpio_num = config.in2_pin;         // Set the GPIO pin for the channel output.
    channel_2_config.speed_mode = LEDC_LOW_SPEED_MODE;  // Have the hardware timer operate in low-speed.
    channel_2_config.channel = config.channel_2;        // Set the LEDC channel.
    channel_2_config.timer_sel = LEDC_TIMER_0;          // Pick the first hardware timer index for this channel to use.
    channel_2_config.duty = 0;                          // Configure the initial duty cycle to 0.
    channel_2_config.hpoint = 0;                        // Set the signal to pull high at the start of each timer cycle.

    err = ledc_channel_config(&channel_2_config);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure motor LEDC channel [2].");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Initialized motor on GPIO Pins [%d] and [%d] with motor's PWM output to [%u].",
            config.in1_pin,
            config.in2_pin,
            current_duty
        );
    state = Motor_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the initialized boolean.
    ============================================================
*/
bool Motor::is_initialized() const {
    return state == Motor_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the faulted boolean.
    ============================================================
*/
bool Motor::is_faulted() const {
    return state == Motor_State::FAULT;
}
//  ============================================================


/*
    ============================================================
    Get the motor's current command.
    ============================================================
*/
Motor_Command Motor::get_motor_command() const {
    return command;
}
//  ============================================================


/*
    ============================================================
    Record the motor's PWM output between 0 to 255.

    NOTE: Callers must validate or limit the calculated duty
    value before converting it to uint8_t. An out-of-range
    integer converted to uint8_t is narrowed to the destination
    type rather than being rejected automatically, which could
    result in unintended motor command.
    ============================================================
*/
void Motor::set_duty_cycle(uint8_t duty) {
    if (state == Motor_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor is not initialized. Ignoring set_duty_cycle().");
        return;
    }
    else if (state == Motor_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor is in faulted state. Ignoring set_duty_cycle().");
        return;
    }

    current_duty = duty;
    ESP_LOGI(config.name.c_str(), "Adjusting motor's PWM output to: [%u].", current_duty);
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the motor's PWM output.
    ============================================================
*/
uint8_t Motor::get_duty_cycle() const {
    return current_duty;
}
//  ============================================================


/*
    ============================================================
    Commands the motor to perform a forward rotation by applying
    the current duty cycle to channel 1 and a zero duty cycle to
    channel 2.
    ============================================================
*/
void Motor::spin_forward() {
    if (state == Motor_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor is not initialized. Ignoring spin_forward().");
        return;
    }
    else if (state == Motor_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor is in faulted state. Ignoring spin_forward().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Spinning motor forward at PWM output: [%u].", current_duty);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, current_duty);
    if (err != ESP_OK) {
        enter_fault("spin_forward(): ledc_set_duty(channel_1)", err);
        return;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);
    if (err != ESP_OK) {
        enter_fault("spin_forward(): ledc_update_duty(channel_1)", err);
        return;
    }

    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, 0);
    if (err != ESP_OK) {
        enter_fault("spin_forward(): ledc_set_duty(channel_2)", err);
        return;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);
    if (err != ESP_OK) {
        enter_fault("spin_forward(): ledc_update_duty(channel_2)", err);
        return;
    }

    command = Motor_Command::FORWARD;
    return;
}
//  ============================================================


/*
    ============================================================
    Commands the motor to perform a backward rotation by
    applying a zero duty cycle to channel 1 and the current duty
    cycle to channel 2.
    ============================================================
*/
void Motor::spin_backward() {
    if (state == Motor_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor is not initialized. Ignoring spin_backward().");
        return;
    }
    else if (state == Motor_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor is in faulted state. Ignoring spin_backward().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Spinning motor backward at PWM output: [%u].", current_duty);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, 0);
    if (err != ESP_OK) {
        enter_fault("spin_backward(): ledc_set_duty(channel_1)", err);
        return;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);
    if (err != ESP_OK) {
        enter_fault("spin_backward(): ledc_update_duty(channel_1)", err);
        return;
    }

    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, current_duty);
    if (err != ESP_OK) {
        enter_fault("spin_backward(): ledc_set_duty(channel_2)", err);
        return;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);
    if (err != ESP_OK) {
        enter_fault("spin_backward(): ledc_update_duty(channel_2)", err);
        return;
    }

    command = Motor_Command::BACKWARD;
    return;
}
//  ============================================================


/*
    ============================================================
    Commands the motor to perform no rotational movement by
    applying a zero duty cycle to the current duty cycle and to
    channels 1 and 2.
    ============================================================
*/
void Motor::stop() {
    if (state == Motor_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor is not initialized. Ignoring stop().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Stopping motor from spinning.");

    current_duty = 0;
    
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, current_duty);
    if (err != ESP_OK) {
        enter_fault("stop(): ledc_set_duty(channel_1)", err);
        return;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);
    if (err != ESP_OK) {
        enter_fault("stop(): ledc_update_duty(channel_1)", err);
        return;
    }

    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, current_duty);
    if (err != ESP_OK) {
        enter_fault("stop(): ledc_set_duty(channel_2)", err);
        return;
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);
    if (err != ESP_OK) {
        enter_fault("stop(): ledc_update_duty(channel_2)", err);
        return;
    }

    command = Motor_Command::STOP;
    return;
}
//  ============================================================


/*
    ============================================================
    Enters the motor into a fault state if an error occurs after
    initialization.
    ============================================================
*/
void Motor::enter_fault(const char* operation, esp_err_t err) {
    ESP_LOGE(
        config.name.c_str(),
        "Motor hardware failure during: [%s]. ESP error: [%s]. Entering fault state.",
        operation,
        esp_err_to_name(err)
    );

    state = Motor_State::FAULT;
    command = Motor_Command::STOP;
    current_duty = 0;

    // Best-effort attempt to remove actuator output.
    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, config.channel_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, config.channel_2);

    return;
}
//  ============================================================