/*
    This file defines the Motor_Driver class used to control 2 motors,
    perform basic movements, and adjust speeds using the L298N motor
    driver.
*/

#ifndef MOTOR_DRIVER_HPP_
#define MOTOR_DRIVER_HPP_

/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include "motors/motor.hpp"

#include <cstdint>

#include <esp_log.h>

#include <string>
//  ============================================================



/*
    ============================================================
    Struct that stores the user-defined configurations for the
    motor driver.
    ============================================================
*/
struct Driver_Config {
    std::string name;
    Motor Left_Motor;
    Motor Right_Motor;
};
//  ============================================================



/*
    ============================================================
    Enum that stores the motor driver's current state.
    ============================================================
*/
enum class Motor_Driver_State {
    UNINITIALIZED,
    READY,
    FAULT
};
//  ============================================================



/*
    ============================================================
    Enum that stores the motor driver's current command.
    ============================================================
*/
enum class Motor_Driver_Command {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT_TURN,
    RIGHT_TURN
};
//  ============================================================



/*
    ============================================================
    This class controls the actuator output and direction of 2
    motors through GPIO and PWM signals.
    ============================================================
*/
class Motor_Driver {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Motor_Driver(const Driver_Config &driver_setup);
        ~Motor_Driver() = default;
        bool is_initialized() const;
        bool is_faulted() const;

        Motor_Driver_Command get_driver_command() const;
        Motor_Command get_left_motor_command() const;
        Motor_Command get_right_motor_command() const;

        void set_left_duty_cycle(uint8_t left_duty);
        uint8_t get_left_duty_cycle() const;

        void set_right_duty_cycle(uint8_t right_duty);
        uint8_t get_right_duty_cycle() const;

        void forward();
        void backward();
        void left_turn();
        void right_turn();
        void stop();

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        bool has_motor_fault() const;
        void enter_fault(const char* operation);

        Driver_Config config;
        Motor_Driver_State state = Motor_Driver_State::UNINITIALIZED;
        Motor_Driver_Command command = Motor_Driver_Command::STOP;
        uint8_t current_left_duty = 0;
        uint8_t current_right_duty = 0;
};
//  ============================================================

#endif