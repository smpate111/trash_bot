/*
    This file fleshes out the Motor_Driver class logic defined in motor_driver.hpp.
*/

/*
    ============================================================
    Include the Motor_Driver class header file to access the
    class's methods and variables.
    ============================================================
*/
#include <../include/motors/motor_driver.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Motor_Driver class that initializes the
    motors with user-defined values.
    ============================================================
*/
Motor_Driver::Motor_Driver(const Driver_Config &driver_setup) : config(driver_setup) {
    ESP_LOGI(config.name.c_str(), "Initialized driver.");
}
//  ============================================================


/*
    ============================================================
    Adjusts the speed for both motors.
    ============================================================
*/
void Motor_Driver::adjust_speed(uint32_t left_speed, uint32_t right_speed) {
    ESP_LOGI(config.name.c_str(), "Adjusting driver's speeds.");

    config.Left_Motor.set_speed(left_speed);
    config.Right_Motor.set_speed(right_speed);
    return;
}
//  ============================================================


/*
    ============================================================
    Drives both motors forward.
    ============================================================
*/
void Motor_Driver::forward() {
    ESP_LOGI(config.name.c_str(), "Making driver move forward.");

    config.Left_Motor.spin_forward();
    config.Right_Motor.spin_forward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives both motors backward.
    ============================================================
*/
void Motor_Driver::backward() {
    ESP_LOGI(config.name.c_str(), "Making driver move backward.");

    config.Left_Motor.spin_backward();
    config.Right_Motor.spin_backward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives the left motor backward and right motor forward to
    make robot turn left.
    ============================================================
*/
void Motor_Driver::left() {
    ESP_LOGI(config.name.c_str(), "Making driver turn left.");

    config.Left_Motor.spin_backward();
    config.Right_Motor.spin_forward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives the left motor forward and right motor backward to
    make robot turn right.
    ============================================================
*/
void Motor_Driver::right() {
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
void Motor_Driver::brake() {
    ESP_LOGI(config.name.c_str(), "Making driver stop.");

    config.Left_Motor.stop();
    config.Right_Motor.stop();
    return;
}
//  ============================================================