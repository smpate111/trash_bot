/*
    This file fleshes out the Drive_Train class logic defined in drive_train.hpp.
*/

/*
    ============================================================
    Include the following header files to access their
    class's methods and variables
    ============================================================
*/
#include "controllers/drive_train.hpp"
//  ============================================================



/*
    ============================================================
    Constructor for the Drive_Train class that initializes the
    motors with user-defined values.
    ============================================================
*/
Drive_Train::Drive_Train(const Train_Config &train_setup) : config(train_setup) {
    if (config.Front_Driver.is_faulted() == true) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize drive train. The Front Motor Driver faulted.");
        state = Train_State::FAULT;
        return;
    }
    else if (config.Front_Driver.is_initialized() == false) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Front Motor Driver.");
        return;
    }
    

    if (config.Rear_Driver.is_faulted() == true) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize drive train. The Rear Motor Driver faulted.");
        state = Train_State::FAULT;
        return;
    }
    else if (config.Rear_Driver.is_initialized() == false) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Rear Motor Driver.");
        return;
    }

    if (config.Left_Encoder.is_faulted() == true) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize drive train. The Left Wheel Encoder faulted.");
        state = Train_State::FAULT;
        return;
    }
    else if (config.Left_Encoder.is_initialized() == false) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Left Wheel Encoder.");
        return;
    }


    if (config.Right_Encoder.is_faulted() == true) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize drive train. The Right Wheel Encoder faulted.");
        state = Train_State::FAULT;
        return;
    }
    else if (config.Right_Encoder.is_initialized() == false) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize the Right Wheel Encoder.");
        return;
    }

    ESP_LOGI(
            config.name.c_str(),
            "Initialized drive train with current left motors' PWM output to [%u] and current right motors' PWM output to [%u].",
            current_left_duty,
            current_right_duty
        );
    state = Train_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the initialized boolean.
    ============================================================
*/
bool Drive_Train::is_initialized() const {
    return state == Train_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the faulted boolean.
    ============================================================
*/
bool Drive_Train::is_faulted() const {
    return state == Train_State::FAULT;
}
//  ============================================================


/*
    ============================================================
    Get the drive train's current command.
    ============================================================
*/
Train_Command Drive_Train::get_train_command() const {
    return command;
}
//  ============================================================


/*
    ============================================================
    Get the front motor driver's current command.
    ============================================================
*/
Motor_Driver_Command Drive_Train::get_front_driver_command() const {
    return config.Front_Driver.get_driver_command();
}
//  ============================================================


/*
    ============================================================
    Get the rear motor driver's current command.
    ============================================================
*/
Motor_Driver_Command Drive_Train::get_rear_driver_command() const {
    return config.Rear_Driver.get_driver_command();
}
//  ============================================================


/*
    ============================================================
    Get the front left motor's current command.
    ============================================================
*/
Motor_Command Drive_Train::get_fd_left_motor_command() const {
    return config.Front_Driver.get_left_motor_command();
}
//  ============================================================

/*
    ============================================================
    Get the front right motor's current command.
    ============================================================
*/
Motor_Command Drive_Train::get_fd_right_motor_command() const {
    return config.Front_Driver.get_right_motor_command();
}
//  ============================================================


/*
    ============================================================
    Get the rear left motor's current command.
    ============================================================
*/
Motor_Command Drive_Train::get_rd_left_motor_command() const {
    return config.Rear_Driver.get_left_motor_command();
}
//  ============================================================

/*
    ============================================================
    Get the rear right motor's current command.
    ============================================================
*/
Motor_Command Drive_Train::get_rd_right_motor_command() const {
    return config.Rear_Driver.get_right_motor_command();
}
//  ============================================================


/*
    ============================================================
    Record the left motors' PWM output between 0 to 255.

    NOTE: Callers must limit the calculated duty value before
    converting it to uint8_t. An out-of-range integer converted
    to uint8_t is narrowed to the destination type rather than
    being rejected automatically.
    ============================================================
*/
void Drive_Train::set_left_duty_cycle(uint8_t left_duty) {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring set_left_duty_cycle().");
        return;
    }
    else if (state == Train_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Drive train is in faulted state. Ignoring set_left_duty_cycle().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train adjust left motors' PWM output to: [%u].", left_duty);
    
    config.Front_Driver.set_left_duty_cycle(left_duty);
    if (has_motor_driver_fault() == true) {
        enter_fault("Front_Driver: set_left_duty_cycle()");
        return;
    }
    
    config.Rear_Driver.set_left_duty_cycle(left_duty);
    if (has_motor_driver_fault() == true) {
        enter_fault("Rear_Driver: set_left_duty_cycle()");
        return;
    }
    
    current_left_duty = left_duty;

    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the left motors' PWM output.
    ============================================================
*/
uint8_t Drive_Train::get_left_duty_cycle() const {
    return current_left_duty;
}
//  ============================================================


/*
    ============================================================
    Record the right motors' PWM output between 0 to 255.

    NOTE: Callers must limit the calculated duty value before
    converting it to uint8_t. An out-of-range integer converted
    to uint8_t is narrowed to the destination type rather than
    being rejected automatically.
    ============================================================
*/
void Drive_Train::set_right_duty_cycle(uint8_t right_duty) {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring set_right_duty_cycle().");
        return;
    }
    else if (state == Train_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Drive train is in faulted state. Ignoring set_right_duty_cycle().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train adjust right motors' PWM output to: [%u].", right_duty);
    
    config.Front_Driver.set_right_duty_cycle(right_duty);
    if (has_motor_driver_fault() == true) {
        enter_fault("Front_Driver: set_right_duty_cycle()");
        return;
    }
    
    config.Rear_Driver.set_right_duty_cycle(right_duty);
    if (has_motor_driver_fault() == true) {
        enter_fault("Rear_Driver: set_right_duty_cycle()");
        return;
    }
    
    current_right_duty = right_duty;

    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the right motors' PWM output.
    ============================================================
*/
uint8_t Drive_Train::get_right_duty_cycle() const {
    return current_right_duty;
}
//  ============================================================


/*
    ============================================================
    Commands front and rear motor drivers to move forward.
    ============================================================
*/
void Drive_Train::forward() {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring forward().");
        return;
    }
    else if (state == Train_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Drive train is in faulted state. Ignoring forward().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train move forward.");

    config.Front_Driver.forward();
    if (has_motor_driver_fault() == true) {
        enter_fault("Front_Driver: forward()");
        return;
    }

    config.Rear_Driver.forward();
    if (has_motor_driver_fault() == true) {
        enter_fault("Rear_Driver: forward()");
        return;
    }

    command = Train_Command::FORWARD;

    return;
}
//  ============================================================


/*
    ============================================================
    Commands front and rear motor drivers to move backward.
    ============================================================
*/
void Drive_Train::backward() {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring backward().");
        return;
    }
    else if (state == Train_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Drive train is in faulted state. Ignoring backward().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train move backward.");

    config.Front_Driver.backward();
    if (has_motor_driver_fault() == true) {
        enter_fault("Front_Driver: backward()");
        return;
    }

    config.Rear_Driver.backward();
    if (has_motor_driver_fault() == true) {
        enter_fault("Rear_Driver: backward()");
        return;
    }

    command = Train_Command::BACKWARD;

    return;
}
//  ============================================================


/*
    ============================================================
    Commands front and rear motor drivers to make a left turn.
    ============================================================
*/
void Drive_Train::left_turn() {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring left_turn().");
        return;
    }
    else if (state == Train_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Drive train is in faulted state. Ignoring left_turn().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train turn left.");

    config.Front_Driver.left_turn();
    if (has_motor_driver_fault() == true) {
        enter_fault("Front_Driver: left_turn()");
        return;
    }

    config.Rear_Driver.left_turn();
    if (has_motor_driver_fault() == true) {
        enter_fault("Rear_Driver: left_turn()");
        return;
    }

    command = Train_Command::LEFT_TURN;

    return;
}
//  ============================================================


/*
    ============================================================
    Commands front and rear motor drivers to make a right turn.
    ============================================================
*/
void Drive_Train::right_turn() {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring right_turn().");
        return;
    }
    else if (state == Train_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Drive train is in faulted state. Ignoring right_turn().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train turn right.");

    config.Front_Driver.right_turn();
    if (has_motor_driver_fault() == true) {
        enter_fault("Front_Driver: right_turn()");
        return;
    }

    config.Rear_Driver.right_turn();
    if (has_motor_driver_fault() == true) {
        enter_fault("Rear_Driver: right_turn()");
        return;
    }

    command = Train_Command::RIGHT_TURN;

    return;
}
//  ============================================================


/*
    ============================================================
    Commands front and rear motor drivers to stop moving.
    ============================================================
*/
void Drive_Train::stop() {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring stop().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train stop.");

    current_left_duty = 0;
    current_right_duty = 0;

    config.Front_Driver.stop();
    config.Rear_Driver.stop();

    command = Train_Command::STOP;

    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the left wheel encoder's pulse count.
    ============================================================
*/
uint32_t Drive_Train::get_left_pulse_count() const {
    return config.Left_Encoder.get_pulse_count();
}
//  ============================================================


/*
    ============================================================
    Retrieve the right wheel encoder's pulse count.
    ============================================================
*/
uint32_t Drive_Train::get_right_pulse_count() const {
    return config.Right_Encoder.get_pulse_count();
}
//  ============================================================


/*
    ============================================================
    Resets both wheel encoders' pulse counts.
    ============================================================
*/
void Drive_Train::reset_encoder_counts() {
    if (state == Train_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Drive train is not initialized. Ignoring reset_encoder_counts().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Making drive train reset both encoders' pulse counts.");
    config.Left_Encoder.reset_count();
    config.Right_Encoder.reset_count();
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the left wheel encoder's calculated distance.
    ============================================================
*/
double Drive_Train::get_left_distance() const {
    return config.Left_Encoder.calculate_distance();
}
//  ============================================================


/*
    ============================================================
    Retrieve the right wheel encoder's calculated distance.
    ============================================================
*/
double Drive_Train::get_right_distance() const {
    return config.Right_Encoder.calculate_distance();
}
//  ============================================================


/*
    ============================================================
    Determine if 1 of the 2 motor drivers entered into a fault
    state.
    ============================================================
*/
bool Drive_Train::has_motor_driver_fault() const {
    return config.Front_Driver.is_faulted() || config.Rear_Driver.is_faulted();
}
//  ============================================================


/*
    ============================================================
    Enters the drive train into a fault state if an error
    occurs after initialization.
    ============================================================
*/
void Drive_Train::enter_fault(const char* operation) {
    ESP_LOGE(
        config.name.c_str(),
        "Drive train fault during: [%s]. One or more motor drivers and/or one or more wheel encoders entered FAULT. Entering fault state.",
        operation
    );

    state = Train_State::FAULT;
    command = Train_Command::STOP;
    current_left_duty = 0;
    current_right_duty = 0;

    // Best-effort attempt to remove actuator output.
    config.Front_Driver.stop();
    config.Rear_Driver.stop();
    config.Left_Encoder.reset_count();
    config.Right_Encoder.reset_count();

    return;
}
//  ============================================================