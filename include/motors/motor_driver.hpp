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

#include <driver/gpio.h>
#include <driver/ledc.h>

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
        Driver_Config config;
        bool initialized = false;
        uint8_t current_left_duty = 0;
        uint8_t current_right_duty = 0;
};
//  ============================================================

#endif