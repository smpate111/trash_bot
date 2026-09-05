/*
    This file defines the Motor class used to control a motor using
    GPIO pins and PWM signals.
*/

#ifndef MOTOR_HPP_
#define MOTOR_HPP_



/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <esp_err.h>
#include <esp_log.h>

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
//  ============================================================



/*
    ============================================================
    Struct that stores the user-defined configurations for the
    motor.
    ============================================================
*/
struct Motor_Config {
    std::string name;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    ledc_channel_t channel_1;
    ledc_channel_t channel_2;
};
//  ============================================================



/*
    ============================================================
    This class manages a motor's speed and direction.
    ============================================================
*/
class Motor {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Motor(const Motor_Config &motor_setup);
        
        virtual ~Motor() = default;

        virtual void set_speed(uint32_t speed);
        uint32_t get_speed();

        virtual void spin_forward();
        virtual void spin_backward();
        virtual void stop();

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        Motor_Config config;
        uint32_t Current_Speed = 0;
};
//  ============================================================

#endif