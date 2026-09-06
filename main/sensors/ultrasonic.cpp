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
    gpio_reset_pin(config.trig_pin);
    gpio_reset_pin(config.echo_pin);
    gpio_set_direction(config.trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.echo_pin, GPIO_MODE_INPUT);

    // Configure the TRIG pin.
    gpio_config_t trigger_config = {};
    trigger_config.pin_bit_mask = (1ULL << config.trig_pin);    // Set the pin's bit mask.
    trigger_config.mode = GPIO_MODE_OUTPUT;                     // Set the pin to be an output.
    trigger_config.pull_up_en = GPIO_PULLUP_DISABLE;            // Disable the pull-up resistor.
    trigger_config.pull_down_en = GPIO_PULLDOWN_DISABLE;        // Disable the pull-down resistor.
    trigger_config.intr_type = GPIO_INTR_DISABLE;               // Disable the interrupt for this pin.

    gpio_config(&trigger_config);
    gpio_set_level(config.trig_pin, 0);    // Initially set the TRIG pin to low.

    // Configure the ECHO pin.
    gpio_config_t echo_config = {};
    echo_config.pin_bit_mask = (1ULL << config.echo_pin);   // Set the pin's bit mask.
    echo_config.mode = GPIO_MODE_INPUT;                     // Set the pin to be an input.
    echo_config.pull_up_en = GPIO_PULLUP_DISABLE;           // Disable the pull-up resistor.
    echo_config.pull_down_en = GPIO_PULLDOWN_DISABLE;       // Disable the pull-down resistor.
    echo_config.intr_type = GPIO_INTR_ANYEDGE;              // Set the interrupt to trigger on both falling and rising edges.

    gpio_config(&echo_config);

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

    uint32_t now = (uint32_t)esp_timer_get_time();
    int level = gpio_get_level(us->get_echo_pin());

    // Start recording the time when ECHO pin is high.
    if (level == 1) {
        us->set_start_echo_time(now);
    }
    // Stop recording the time and calculate the object's distance when ECHO pin is low.
    else {
        if (us->get_start_echo_time() > 0) {
            uint32_t duration = now - us->get_start_echo_time();
            us->set_distance((static_cast<double>(duration) * 0.343) / 2.0);
            us->set_start_echo_time(0);
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
    // Check if we are testing so that we don't get stuck in an infinite loop.
    if (is_testing == true) {
        return;
    }

    // Reset ECHO signal duration before sending new signal.
    set_start_echo_time(0);
    set_distance(-1.0);

    // Send TRIG pulse for 10us. This will trigger the ISR above.
    gpio_set_level(get_trig_pin(), 1);
    esp_rom_delay_us(10);
    gpio_set_level(get_trig_pin(), 0);

    // Calculate timeout in ticks.
    TickType_t timeout_ticks = pdMS_TO_TICKS(30);
    TickType_t start_tick = xTaskGetTickCount();

    // Sleep until the ISR calculates a positive distance or we reach timeout.
    while ((get_distance() < 0) && (xTaskGetTickCount() - start_tick < timeout_ticks)) {
        vTaskDelay(1);
    }

    // Print log based on whether timeout was reached or we detected an object.
    if (get_distance() < 0) {
        set_distance(-1.0);
        ESP_LOGI(config.name.c_str(), "Signal was lost or out of range.");
    }
    else {
        ESP_LOGI(config.name.c_str(), "Object detected at %0.4fmm.", get_distance());
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve calculated distance.
    ============================================================
*/
double Ultrasonic::get_distance() {
    return distance;
}
//  ============================================================


/*
    ============================================================
    Record calculated distance.
    ============================================================
*/
void Ultrasonic::set_distance(double dist) {
    distance = dist;
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve echo's start time.
    ============================================================
*/
uint32_t Ultrasonic::get_start_echo_time() {
    return start_echo_time;
}
//  ============================================================


/*
    ============================================================
    Record echo's start time.
    ============================================================
*/
void Ultrasonic::set_start_echo_time(uint32_t time) {
    start_echo_time = time;
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve echo pin.
    ============================================================
*/
gpio_num_t Ultrasonic::get_echo_pin() {
    return config.echo_pin;
}
//  ============================================================


/*
    ============================================================
    Retrieve trig pin.
    ============================================================
*/
gpio_num_t Ultrasonic::get_trig_pin() {
    return config.trig_pin;
}
//  ============================================================