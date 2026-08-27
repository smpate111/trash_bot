/*
    This file fleshes out the Drive_Train class logic defined in drive_train.hpp.
*/

/*
    ============================================================
    Include the following header files to access their
    class's methods and variables
    ============================================================
*/
#include <../include/controllers/drive_train.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Drive_Train class that initializes the
    motors with user-defined values.
    ============================================================
*/
Drive_Train::Drive_Train(const Train_Config &train_setup) : config(train_setup) {
    ESP_LOGI(config.name.c_str(), "Initialized drive train.");
}
//  ============================================================


/*
    ============================================================
    Adjusts the speed for all 4 motors.
    ============================================================
*/
void Drive_Train::change_speed(uint32_t left_speed, uint32_t right_speed) {
    ESP_LOGI(config.name.c_str(), "Adjusting the train's speed.");

    config.Front_Driver.adjust_speed(left_speed, right_speed);
    config.Back_Driver.adjust_speed(left_speed, right_speed);
    return;
}
//  ============================================================


/*
    ============================================================
    Drives all motors forward.
    ============================================================
*/
void Drive_Train::move_forward() {
    ESP_LOGI(config.name.c_str(), "Making train move forward.");

    config.Front_Driver.forward();
    config.Back_Driver.forward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives all motors backward.
    ============================================================
*/
void Drive_Train::move_backward() {
    ESP_LOGI(config.name.c_str(), "Making train move backward.");

    config.Front_Driver.backward();
    config.Back_Driver.backward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives the 2 left motors backward and 2 right motors forward
    to make robot turn left.
    ============================================================
*/
void Drive_Train::turn_left() {
    ESP_LOGI(config.name.c_str(), "Making train turn left.");

    config.Front_Driver.left();
    config.Back_Driver.left();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives the 2 left motors forward and 2 right motors backward
    to make robot turn right.
    ============================================================
*/
void Drive_Train::turn_right() {
    ESP_LOGI(config.name.c_str(), "Making train turn right.");

    config.Front_Driver.right();
    config.Back_Driver.right();
    return;
}
//  ============================================================


/*
    ============================================================
    Stops all motors from spinning.
    ============================================================
*/
void Drive_Train::brake_all() {
    ESP_LOGI(config.name.c_str(), "Making train stop.");

    config.Front_Driver.brake();
    config.Back_Driver.brake();
    return;
}
//  ============================================================


/*
    ============================================================
    Returns the left wheel encoder object.
    ============================================================
*/
Wheel_Encoder& Drive_Train::get_left_encoder() {
    //ESP_LOGI(config.name.c_str(), "Retrieving left wheel encoder object.");
    return *(config.Left_Encoder);  // Returning a dereferenced pointer.
}
//  ============================================================


/*
    ============================================================
    Returns the right wheel encoder object.
    ============================================================
*/
Wheel_Encoder& Drive_Train::get_right_encoder() {
    //ESP_LOGI(config.name.c_str(), "Retrieving right wheel encoder object.");
    return *(config.Right_Encoder); // Returning a dereferenced pointer.
}
//  ============================================================