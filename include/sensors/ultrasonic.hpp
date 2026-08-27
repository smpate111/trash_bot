/*
    This file defines the Ultrasonic class used to control an ultrasonic sensor using
    GPIO pins and PWM signals.
*/

#ifndef ULTRASONIC_HPP_
#define ULTRASONIC_HPP_



/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

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
    ultrasonic sensor.
    ============================================================
*/
struct Ultrasonic_Config {
    std::string name;
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;
};
//  ============================================================



/*
    ============================================================
    This class manages the sensor's ticks.
    ============================================================
*/
class Ultrasonic {
    // Set these methods to public to allow access and control from outside the class.
    public:
        Ultrasonic(const Ultrasonic_Config &ultrasonic_setup);
        virtual ~Ultrasonic() = default;

        static void IRAM_ATTR isr_handler(void *arg);

        virtual void measure_distance();

        std::string Ultrasonic_Name;
        volatile double distance = 1000.0;      // Initially set this to prevent errors.

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        Ultrasonic_Config config;
        volatile int64_t start_echo = 0;
};
//  ============================================================

#endif