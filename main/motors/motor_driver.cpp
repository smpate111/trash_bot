/*
    This file fleshes out the Motor_Driver class logic defined in motor_driver.hpp.
*/

/*
    ============================================================
    Include the Motor_Driver class header file to access the
    class's methods and variables.
    ============================================================
*/
#include "motors/motor_driver.hpp"
//  ============================================================



/*
    ============================================================
    Constructor for the Motor_Driver class that initializes the
    motors with user-defined values.
    ============================================================
*/
Motor_Driver::Motor_Driver(const Driver_Config &driver_setup) : config(driver_setup) {
    if (config.Left_Motor.is_initialized() == false) {
        initialized = false;
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Left Motor.");
        return;
    }
    else if (config.Right_Motor.is_initialized() == false) {
        initialized = false;
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Right Motor.");
        return;
    }

    initialized = true;
    ESP_LOGI(config.name.c_str(), "Initialized motor driver.");
}
//  ============================================================


/*
    ============================================================
    Retrieves the initialized boolean.
    ============================================================
*/
bool Motor_Driver::is_initialized() const {
    return initialized;
}
//  ============================================================


/*
    ============================================================
    Record the left motor's PWM output between 0 to 255.

    NOTE: Callers must limit the calculated duty value before
    converting it to uint8_t. An out-of-range integer converted
    to uint8_t is narrowed to the destination type rather than
    being rejected automatically.
    ============================================================
*/
void Motor_Driver::set_left_duty_cycle(uint8_t left_duty) {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Left motor is not initialized. Ignoring set_left_duty_cycle().");
        return;
    }

    config.Left_Motor.set_duty_cycle(left_duty);
    current_left_duty = config.Left_Motor.get_duty_cycle();
    ESP_LOGI(config.name.c_str(), "Adjusted left motor's PWM output to: [%u].", current_left_duty);
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the left motor's PWM output.
    ============================================================
*/
uint8_t Motor_Driver::get_left_duty_cycle() const {
    return current_left_duty;
}
//  ============================================================


/*
    ============================================================
    Record the right motor's PWM output between 0 to 255.

    NOTE: Callers must limit the calculated duty value before
    converting it to uint8_t. An out-of-range integer converted
    to uint8_t is narrowed to the destination type rather than
    being rejected automatically.
    ============================================================
*/
void Motor_Driver::set_right_duty_cycle(uint8_t right_duty) {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Right motor is not initialized. Ignoring set_right_duty_cycle().");
        return;
    }

    config.Right_Motor.set_duty_cycle(right_duty);
    current_right_duty = config.Right_Motor.get_duty_cycle();
    ESP_LOGI(config.name.c_str(), "Adjusted right motor's PWM output to: [%u].", current_right_duty);
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the right motor's PWM output.
    ============================================================
*/
uint8_t Motor_Driver::get_right_duty_cycle() const {
    return current_right_duty;
}
//  ============================================================


/*
    ============================================================
    Spin both motors forward.
    ============================================================
*/
void Motor_Driver::forward() {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring forward().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making driver move forward.");
    config.Left_Motor.spin_forward();
    config.Right_Motor.spin_forward();
    return;
}
//  ============================================================


/*
    ============================================================
    Spin both motors backward.
    ============================================================
*/
void Motor_Driver::backward() {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring backward().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making driver move backward.");
    config.Left_Motor.spin_backward();
    config.Right_Motor.spin_backward();
    return;
}
//  ============================================================


/*
    ============================================================
    Spins the left motor backward and spins the right motor
    forward to make the robot turn left.
    ============================================================
*/
void Motor_Driver::left_turn() {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring left_turn().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making driver turn left.");
    config.Left_Motor.spin_backward();
    config.Right_Motor.spin_forward();
    return;
}
//  ============================================================


/*
    ============================================================
    Spins the left motor forward and spins the right motor
    backward to make the robot turn right.
    ============================================================
*/
void Motor_Driver::right_turn() {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring right_turn().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making driver turn right.");
    config.Left_Motor.spin_forward();
    config.Right_Motor.spin_backward();
    return;
}
//  ============================================================


/*
    ============================================================
    Stops both motors from spinning.
    ============================================================
*/
void Motor_Driver::stop() {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring stop().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making driver stop.");
    set_left_duty_cycle(0);
    set_right_duty_cycle(0);
    config.Left_Motor.stop();
    config.Right_Motor.stop();
    return;
}
//  ============================================================