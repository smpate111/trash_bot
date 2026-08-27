/*
    This file fleshes out the Ultrasonic class logic defined in ultrasonic.hpp.
*/

/*
    ============================================================
    Include the Ultrasonic class header file to access
    the class's methods and variables.
    ============================================================
*/
#include <../include/sensors/ultrasonic.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Ultrasonic class that initializes
    the ultrasonic sensor with user-defined values.
    ============================================================
*/
Ultrasonic::Ultrasonic(const Ultrasonic_Config &ultrasonic_setup) : config(ultrasonic_setup) {

    // Set the ECHO and TRIG pins to input and output respectively.
    gpio_reset_pin(config.echo_pin);
    gpio_reset_pin(config.trig_pin);
    gpio_set_direction(config.echo_pin, GPIO_MODE_INPUT);
    gpio_set_direction(config.trig_pin, GPIO_MODE_OUTPUT);

    // Configure the ECHO pin.
    gpio_config_t echo_config = {};
    echo_config.pin_bit_mask = (1ULL << config.echo_pin);   // Set the pin's bit mask.
    echo_config.mode = GPIO_MODE_INPUT;                     // Set the pin to be an input.
    echo_config.pull_up_en = GPIO_PULLUP_DISABLE;           // Disable the pull-up resistor.
    echo_config.pull_down_en = GPIO_PULLDOWN_DISABLE;       // Disable the pull-down resistor.
    echo_config.intr_type = GPIO_INTR_ANYEDGE;              // Set the interrupt to trigger on both falling and rising edges.

    gpio_config(&echo_config);

    // Configure the TRIG pin.
    gpio_config_t trigger_config = {};
    trigger_config.pin_bit_mask = (1ULL << config.trig_pin);    // Set the pin's bit mask.
    trigger_config.mode = GPIO_MODE_OUTPUT;                     // Set the pin to be an output.
    trigger_config.pull_up_en = GPIO_PULLUP_DISABLE;            // Disable the pull-up resistor.
    trigger_config.pull_down_en = GPIO_PULLDOWN_DISABLE;        // Disable the pull-down resistor.
    trigger_config.intr_type = GPIO_INTR_DISABLE;               // Disable the interrupt for this pin.

    gpio_config(&trigger_config);
    gpio_set_level(config.trig_pin, 0);    // Initially set the TRIG pin to low.

    ESP_LOGI(
        config.name.c_str(),
        "Initialized ultrasonic sensor on pins %d (TRIG) and %d (ECHO). Set TRIG pin to low.",
        config.trig_pin,
        config.echo_pin
    );

    gpio_isr_handler_add(config.echo_pin, isr_handler, this);
    ESP_LOGI(config.name.c_str(), "Initialized ISR for ultrasonic sensor.");
}


/*
    ============================================================
    Interrupt Service Routing that records the signal
    from the trigger to hitting the detected to the signal
    coming back to the sensor. This will allow us to record
    the distance from the sensor to the object.
    ============================================================
*/
void IRAM_ATTR Ultrasonic::isr_handler(void *arg) {
    Ultrasonic *us = static_cast<Ultrasonic*>(arg);

    int64_t now = esp_timer_get_time();
    int level = gpio_get_level(us->config.echo_pin);

    // Start recording the time when ECHO pin is high.
    if (level == 1) {
        us->start_echo = now;
    }
    // Stop recording the time and calculate the object's distance when ECHO pin is low.
    else {
        if (us->start_echo > 0) {
            int64_t duration = now - us->start_echo;
            us->distance = (static_cast<double>(duration) * 0.343) / 2.0;
            us->start_echo = 0;
        }
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Measures the object's distance from the sensor by having
    the TRIG pin quickly turned on and off which would trigger
    the interrupt service routine.
    ============================================================
*/
void Ultrasonic::measure_distance() {
    // Reset ECHO signal duration before sending new signal.
    start_echo = 0;

    // Send TRIG pulse for 10us.
    gpio_set_level(config.trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(config.trig_pin, 0);

    // Give ECHO signal 30ms to return.
    int wait_us = 0;
    while ((start_echo == 0) && (wait_us < 30000)) {
        esp_rom_delay_us(100);
        wait_us += 100;
    }

    // Wait until ECHO signal returns or timeout is reached.
    while ((gpio_get_level(config.echo_pin) == 1) && (wait_us < 30000)) {
        esp_rom_delay_us(100);
        wait_us += 100;
    }

    if ((wait_us >= 30000) || (distance <= 0)) {
        distance = 1000.0;
        ESP_LOGI(config.name.c_str(), "Signal was lost or out of range.");
    }
    else {
        ESP_LOGI(config.name.c_str(), "Object detected at %0.4fmm.", distance);
    }

    return;
}
//  ============================================================