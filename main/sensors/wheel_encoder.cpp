/*
    This file fleshes out the Wheel Encoder class logic defined in wheel_encoder.hpp.
*/

/*
    ============================================================
    Include the Wheel Encoderclass header file to access the
    class's methods and variables.
    ============================================================
*/
#include "sensors/wheel_encoder.hpp"
//  ============================================================



/*
    ============================================================
    Constructor for the Wheel Encoder class that initializes the
    wheel encoder sensor with user-defined values.
    ============================================================
*/
Wheel_Encoder::Wheel_Encoder(const Encoder_Config &encoder_setup) : config(encoder_setup) {
    // Configure the GPIO pin to be an input pin.
    esp_err_t err = gpio_reset_pin(config.out_pin);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to reset GPIO Pin [%d].", config.out_pin);
        return;
    }

    err = gpio_set_direction(config.out_pin, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure GPIO Pin [%d] as input.", config.out_pin);
        return;
    }

    // Ensure the encoder slots and wheel diameter are greater than 0 or else the calculations would not work.
    if (config.encoder_slots <= 0) {
        ESP_LOGW(config.name.c_str(), "Initialization failure. # of encoder slots is less than or equal to 0.");
        return;
    }
    if (config.wheel_diameter <= 0) {
        ESP_LOGW(config.name.c_str(), "Initialization failure. Wheel diameter is less than or equal to 0mm.");
        return;
    }

    // Configure the sensor.
    gpio_config_t wheel_encoder_config = {};
    wheel_encoder_config.pin_bit_mask = (1ULL << config.out_pin);   // Set the pin's bit mask.
    wheel_encoder_config.mode = GPIO_MODE_INPUT;                    // Have the sensor input data.
    wheel_encoder_config.pull_up_en = GPIO_PULLUP_ENABLE;           // Enable the pull-up resistor.
    wheel_encoder_config.pull_down_en = GPIO_PULLDOWN_DISABLE;      // Disable the pull-down resistor.
    wheel_encoder_config.intr_type = GPIO_INTR_NEGEDGE;             // Configure the interrupt to trigger on falling edge (1 to 0).
    
    err = gpio_config(&wheel_encoder_config);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure wheel encoder GPIO.");
        return;
    }

    // Configure the ISR.
    err = gpio_isr_handler_add(config.out_pin, isr_handler, this);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize wheel encoder ISR.");
        return;
    }

    ESP_LOGI(
        config.name.c_str(),
        "Initialized wheel encoder on GPIO Pin [%d], set wheel diameter to [%0.4fmm], set # of encoder slots to [%d], and initialized ISR.",
        config.out_pin,
        config.wheel_diameter,
        config.encoder_slots
    );
    initialized = true;
}
//  ============================================================


/*
    ============================================================
    Retrieves the initialized boolean.
    ============================================================
*/
bool Wheel_Encoder::is_initialized() const {
    return initialized;
}
//  ============================================================


/*
    ============================================================
    Interrupt Service Routine that increases the pulse count
    when a falling edge is detected by the sensor.
    ============================================================
*/
void IRAM_ATTR Wheel_Encoder::isr_handler(void *arg) {
    auto *we = static_cast<Wheel_Encoder*>(arg);
    we->pulse_count.fetch_add(1, std::memory_order_relaxed);
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve the wheel encoder's pulse count.
    ============================================================
*/
uint32_t Wheel_Encoder::get_pulse_count() const {
    return pulse_count.load(std::memory_order_relaxed);
}
//  ============================================================


/*
    ============================================================
    Record the wheel encoder's pulse count.
    ============================================================
*/
void Wheel_Encoder::set_pulse_count(uint32_t count) {
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Wheel encoder is not initialized. Ignoring set_pulse_count().");
        return;
    }

    //ESP_LOGI(config.name.c_str(), "Pulse count is set to to: [%u].", count);
    pulse_count.store(count, std::memory_order_relaxed);
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
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Wheel encoder is not initialized. Ignoring reset_count().");
        return;
    }

    ESP_LOGI(config.name.c_str(), "Total pulses counted is: [%u]. Resetting count to: [0].", pulse_count.load(std::memory_order_relaxed));
    pulse_count.store(0, std::memory_order_relaxed);
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
    if (initialized == false) {
        ESP_LOGW(config.name.c_str(), "Wheel encoder is not initialized. Ignoring calculate_distance().");
        return 0.0;
    }

    double distance = pulse_count * ((PI * config.wheel_diameter) / config.encoder_slots);
    //ESP_LOGI(config.name.c_str(), "Distance traveled is: [%0.4fmm].", distance);
    return distance;
}
//  ============================================================