/*
    This file defines the Motor class used to control a motor using GPIO pins and PWM signals.
*/

#ifndef MOTOR_HPP_
#define MOTOR_HPP_



/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <cstdint>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <esp_log.h>

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
    This class controls the actuator output and direction of a
    single motor through GPIO and PWM signals.
    ============================================================
*/
class Motor {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Motor(const Motor_Config &motor_setup);
        ~Motor() = default;
        bool is_initialized() const;

        void set_duty_cycle(uint8_t duty);
        uint8_t get_duty_cycle() const;

        void spin_forward();
        void spin_backward();
        void stop();

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        const Motor_Config config;
        bool initialized = false;
        uint8_t current_duty = 0;
};
//  ============================================================

#endif