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
    if (config.Left_Motor.is_faulted() == true) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize motor driver. The Left Motor faulted.");
        state = Motor_Driver_State::FAULT;
        return;
    }
    else if (config.Left_Motor.is_initialized() == false) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Left Motor.");
        return;
    }
    

    if (config.Right_Motor.is_faulted() == true) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize motor driver. The Right Motor faulted.");
        state = Motor_Driver_State::FAULT;
        return;
    }
    else if (config.Right_Motor.is_initialized() == false) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Right Motor.");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Initialized motor driver with current left motor's PWM output to [%u] and current right motor's PWM output to [%u].",
            current_left_duty,
            current_right_duty
        );
    state = Motor_Driver_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the initialized boolean.
    ============================================================
*/
bool Motor_Driver::is_initialized() const {
    return state == Motor_Driver_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the faulted boolean.
    ============================================================
*/
bool Motor_Driver::is_faulted() const {
    return state == Motor_Driver_State::FAULT;
}
//  ============================================================


/*
    ============================================================
    Get the motor driver's current command.
    ============================================================
*/
Motor_Driver_Command Motor_Driver::get_driver_command() const {
    return command;
}
//  ============================================================


/*
    ============================================================
    Get the left motor's current command.
    ============================================================
*/
Motor_Command Motor_Driver::get_left_motor_command() const {
    return config.Left_Motor.get_motor_command();
}
//  ============================================================


/*
    ============================================================
    Get the right motor's current command.
    ============================================================
*/
Motor_Command Motor_Driver::get_right_motor_command() const {
    return config.Right_Motor.get_motor_command();
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
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring set_left_duty_cycle().");
        return;
    }
    else if (state == Motor_Driver_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor driver is in faulted state. Ignoring set_left_duty_cycle().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making motor driver adjust left motor's PWM output to: [%u].", left_duty);
    
    config.Left_Motor.set_duty_cycle(left_duty);
    if (has_motor_fault() == true) {
        enter_fault("Left_Motor: set_duty_cycle()");
        return;
    }
    
    current_left_duty = left_duty;

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
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring set_right_duty_cycle().");
        return;
    }
    else if (state == Motor_Driver_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor driver is in faulted state. Ignoring set_right_duty_cycle().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making motor driver adjust right motor's PWM output to: [%u].", right_duty);
    
    config.Right_Motor.set_duty_cycle(right_duty);
    if (has_motor_fault() == true) {
        enter_fault("Right_Motor: set_duty_cycle()");
        return;
    }
    
    current_right_duty = right_duty;
    
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
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring forward().");
        return;
    }
    else if (state == Motor_Driver_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor driver is in faulted state. Ignoring forward().");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Making motor driver move forward with left motor PWM output [%u] and right motor PWM output [%u].",
            current_left_duty,
            current_right_duty
        );

    config.Left_Motor.spin_forward();
    if (has_motor_fault() == true) {
        enter_fault("Left_Motor: spin_forward()");
        return;
    }

    config.Right_Motor.spin_forward();
    if (has_motor_fault() == true) {
        enter_fault("Right_Motor: spin_forward()");
        return;
    }

    command = Motor_Driver_Command::FORWARD;

    return;
}
//  ============================================================


/*
    ============================================================
    Spin both motors backward.
    ============================================================
*/
void Motor_Driver::backward() {
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring backward().");
        return;
    }
    else if (state == Motor_Driver_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor driver is in faulted state. Ignoring backward().");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Making motor driver move backward with left motor PWM output [%u] and right motor PWM output [%u].",
            current_left_duty,
            current_right_duty
        );

    config.Left_Motor.spin_backward();
    if (has_motor_fault() == true) {
        enter_fault("Left_Motor: spin_backward()");
        return;
    }

    config.Right_Motor.spin_backward();
    if (has_motor_fault() == true) {
        enter_fault("Right_Motor: spin_backward()");
        return;
    }

    command = Motor_Driver_Command::BACKWARD;

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
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring left_turn().");
        return;
    }
    else if (state == Motor_Driver_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor driver is in faulted state. Ignoring left_turn().");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Making motor driver turn left with left motor PWM output [%u] and right motor PWM output [%u].",
            current_left_duty,
            current_right_duty
        );

    config.Left_Motor.spin_backward();
    if (has_motor_fault() == true) {
        enter_fault("Left_Motor: spin_backward()");
        return;
    }

    config.Right_Motor.spin_forward();
    if (has_motor_fault() == true) {
        enter_fault("Right_Motor: spin_forward()");
        return;
    }

    command = Motor_Driver_Command::LEFT_TURN;

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
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring right_turn().");
        return;
    }
    else if (state == Motor_Driver_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Motor driver is in faulted state. Ignoring right_turn().");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Making motor driver turn right with left motor PWM output [%u] and right motor PWM output [%u].",
            current_left_duty,
            current_right_duty
        );

    config.Left_Motor.spin_forward();
    if (has_motor_fault() == true) {
        enter_fault("Left_Motor: spin_forward()");
        return;
    }

    config.Right_Motor.spin_backward();
    if (has_motor_fault() == true) {
        enter_fault("Right_Motor: spin_backward()");
        return;
    }

    command = Motor_Driver_Command::RIGHT_TURN;

    return;
}
//  ============================================================


/*
    ============================================================
    Stops both motors from spinning.
    ============================================================
*/
void Motor_Driver::stop() {
    if (state == Motor_Driver_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Motor driver is not initialized. Ignoring stop().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making motor driver stop.");

    current_left_duty = 0;
    current_right_duty = 0;

    config.Left_Motor.stop();
    config.Right_Motor.stop();

    command = Motor_Driver_Command::STOP;

    return;
}
//  ============================================================


/*
    ============================================================
    Determine if 1 of the 2 motors entered into a fault state.
    ============================================================
*/
bool Motor_Driver::has_motor_fault() const {
    return config.Left_Motor.is_faulted() || config.Right_Motor.is_faulted();
}
//  ============================================================


/*
    ============================================================
    Enters the motor driver into a fault state if an error
    occurs after initialization.
    ============================================================
*/
void Motor_Driver::enter_fault(const char* operation) {
    ESP_LOGE(
        config.name.c_str(),
        "Motor driver fault during: [%s]. One or more motors entered FAULT. Entering fault state.",
        operation
    );

    state = Motor_Driver_State::FAULT;
    command = Motor_Driver_Command::STOP;
    current_left_duty = 0;
    current_right_duty = 0;

    // Best-effort attempt to remove actuator output.
    config.Left_Motor.stop();
    config.Right_Motor.stop();

    return;
}
//  ============================================================