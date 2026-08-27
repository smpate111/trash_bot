/*
    This file fleshes out the Wheel Encoder class logic defined in wheel_encoder.hpp.
*/

/*
    ============================================================
    Include the Wheel Encoderclass header file to access the
    class's methods and variables.
    ============================================================
*/
#include <../include/sensors/wheel_encoder.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Wheel Encoder class that initializes the
    wheel encoder sensor with user-defined values and configures
    the timer and channels.
    ============================================================
*/
Wheel_Encoder::Wheel_Encoder(const Encoder_Config &encoder_setup) : config(encoder_setup) {
    // Configure the GPIO pin to be an input pin.
    gpio_reset_pin(config.out_pin);
    gpio_set_direction(config.out_pin, GPIO_MODE_INPUT);

    // Configure the sensor.
    gpio_config_t wheel_encoder_config = {};
    wheel_encoder_config.pin_bit_mask = (1ULL << config.out_pin);   // Set the pin's bit mask.
    wheel_encoder_config.mode = GPIO_MODE_INPUT;                    // Have the sensor input data.
    wheel_encoder_config.pull_up_en = GPIO_PULLUP_ENABLE;           // Enable the pull-up resistor.
    wheel_encoder_config.pull_down_en = GPIO_PULLDOWN_DISABLE;      // Disable the pull-down resistor.
    wheel_encoder_config.intr_type = GPIO_INTR_NEGEDGE;             // Configure the interrupt to trigger on falling edge (1 to 0).
    
    gpio_config(&wheel_encoder_config);
    ESP_LOGI(
        config.name.c_str(),
        "Initialized wheel encoder on pin %d, set wheel diameter to %0.4fmm, set # of encoder slots to %d.",
        config.out_pin,
        config.wheel_diameter,
        config.encoder_slots
    );

    gpio_isr_handler_add(config.out_pin, isr_handler, this);
    ESP_LOGI(config.name.c_str(), "Initialized ISR for wheel encoder.");
}


/*
    ============================================================
    Interrupt Service Routine that increases the pulse count
    when a falling edge is detected by the sensor.
    ============================================================
*/
void IRAM_ATTR Wheel_Encoder::isr_handler(void *arg) {
    Wheel_Encoder *we = static_cast<Wheel_Encoder*>(arg);
    we->pulse_count = we->pulse_count + 1;
    return;
}
//  ============================================================


/*
    ============================================================
    Prints the total pulses counted before resetting the count
    back to 0.
    ============================================================
*/
void Wheel_Encoder::reset_count() {
    ESP_LOGI(config.name.c_str(), "Total pulses counted is: %lu. Resetting count to 0.", pulse_count);
    pulse_count = 0;
    return;
}
//  ============================================================


/*
    ============================================================
    Calculates the distance the motor traveled based on the
    number of encoder slots, the diameter of the wheel, and
    pulse count.
    ============================================================
*/
double Wheel_Encoder::calculate_distance() {
    double distance = pulse_count * ((3.142857 * config.wheel_diameter) / config.encoder_slots);
    //ESP_LOGI(config.name.c_str(), "Distance traveled is: %0.4fmm.", distance);
    return distance;
}
//  ============================================================


/*
    ============================================================
    Measures the motor's velocity by dividing the difference of
    the current and previous pulse count by the number of
    encoder slots and then multiplying it by 60 seconds to get
    the revolutions per minute.
    ============================================================
*/
double Wheel_Encoder::measure_velocity() {
    pulse_difference = pulse_count - last_pulse_count;
    last_pulse_count = pulse_count;
    return static_cast<double>((pulse_difference / config.encoder_slots) * 60.0);
}
//  ============================================================