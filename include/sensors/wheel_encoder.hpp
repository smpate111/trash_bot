/*
    This file defines the Wheel Encoder class used to control a wheel encoder using
    GPIO pins and PWM signals.
*/

#ifndef WHEEL_ENCODER_HPP_
#define WHEEL_ENCODER_HPP_



/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <esp_err.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
//  ============================================================



/*
    ============================================================
    Struct that stores the user-defined configurations for the
    wheel encoder sensor.
    ============================================================
*/
struct Encoder_Config {
    std::string name;
    gpio_num_t out_pin;
    double wheel_diameter;  // in mm
    int encoder_slots;
};
//  ============================================================



/*
    ============================================================
    This class manages an encoder's ticks and direction.
    ============================================================
*/
class Wheel_Encoder {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Wheel_Encoder(const Encoder_Config &encoder_setup);

        virtual ~Wheel_Encoder() = default;

        static void IRAM_ATTR isr_handler(void *arg);

        virtual void reset_count();
        virtual double calculate_distance();
        //virtual double measure_velocity();

        uint32_t get_pulse_count() const;
        void set_pulse_count(uint32_t count);

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        Encoder_Config config;
        volatile uint32_t pulse_count = 0;
        volatile uint32_t last_pulse_count = 0;
        volatile uint32_t pulse_difference = 0;
};
//  ============================================================

#endif