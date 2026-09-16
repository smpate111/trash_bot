/*
    This file defines the Wheel Encoder class used to control a wheel encoder using a GPIO pin.
*/

#ifndef WHEEL_ENCODER_HPP_
#define WHEEL_ENCODER_HPP_



/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <atomic>
#include <cstdint>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
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
    Enum that stores the wheel encoder's current state.
    ============================================================
*/
enum class Encoder_State {
    UNINITIALIZED,
    READY,
    FAULT
};
//  ============================================================



/*
    ============================================================
    This class manages an encoder's pulse count and distance
    measurement.
    ============================================================
*/
class Wheel_Encoder {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Wheel_Encoder(const Encoder_Config &encoder_setup);
        ~Wheel_Encoder() = default;
        bool is_initialized() const;
        bool is_faulted() const;

        static void IRAM_ATTR isr_handler(void *arg);

        uint32_t get_pulse_count() const;
        void set_pulse_count(uint32_t count);
        void reset_count();

        double calculate_distance();

        static constexpr double PI = 3.14159265358979323846;

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        void enter_fault(const char* operation, esp_err_t err);

        const Encoder_Config config;
        Encoder_State state = Encoder_State::UNINITIALIZED;
        std::atomic<uint32_t> pulse_count{0};
        
};
//  ============================================================

#endif