/*
    This file defines the Ultrasonic class used to control an ultrasonic sensor using GPIO pins.
*/

#ifndef ULTRASONIC_HPP_
#define ULTRASONIC_HPP_



/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <atomic>
#include <cstdint>

#include <driver/gpio.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
    Enum that stores the ultrasonic sensor's current state.
    ============================================================
*/
enum class Ultrasonic_State {
    UNINITIALIZED,
    READY,
    FAULT
};
//  ============================================================



/*
    ============================================================
    This class manages the sensor's ticks.
    ============================================================
*/
class Ultrasonic_Sensor {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Ultrasonic_Sensor(const Ultrasonic_Config &ultrasonic_setup);
        ~Ultrasonic_Sensor() = default;
        bool is_initialized() const;
        bool is_faulted() const;

        static void IRAM_ATTR isr_handler(void *arg);

        bool measure_distance();
        double calculate_distance(int64_t duration_us);
        double get_distance() const;

        int64_t get_echo_start_time() const;
        void set_echo_start_time(int64_t time);

        gpio_num_t get_echo_pin();
        gpio_num_t get_trig_pin();

        bool is_testing = false;

        static constexpr double SPEED_OF_SOUND_MM_PER_US = 0.343;
        static constexpr uint32_t MEASUREMENT_TIMEOUT_MS = 30;


    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        bool reset_measurement();
        bool trigger_sensor();
        bool wait_for_echo();
        void enter_fault(const char* operation, esp_err_t err);

        const Ultrasonic_Config config;
        Ultrasonic_State state = Ultrasonic_State::UNINITIALIZED;
        
        double distance_mm = -1.0;      // Initially set this to prevent errors.
        
        std::atomic<bool> measurement_active{false};
        
        std::atomic<int64_t> echo_start_time_us{0};
        std::atomic<int64_t> echo_duration_us{0};
};
//  ============================================================

#endif