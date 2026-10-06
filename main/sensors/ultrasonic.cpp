/*
    This file fleshes out the Ultrasonic class logic defined in ultrasonic.hpp.
*/

/*
    ============================================================
    Include the Ultrasonic class header file to access
    the class's methods and variables.
    ============================================================
*/
#include "sensors/ultrasonic.hpp"
//  ============================================================



/*
    ============================================================
    Constructor for the Ultrasonic class that initializes
    the ultrasonic sensor with user-defined values.
    ============================================================
*/
Ultrasonic_Sensor::Ultrasonic_Sensor(const Ultrasonic_Config &ultrasonic_setup) : config(ultrasonic_setup) {
    if (config.echo_pin == config.trig_pin) {
        ESP_LOGW(
                config.name.c_str(),
                "The trigger GPIO Pin [%d] and the echo GPIO Pin [%d] are the same. They must be different.",
                config.trig_pin,
                config.echo_pin
            );
        return;
    }
    
    // Set the ECHO and TRIG pins to input and output respectively.
    esp_err_t err = gpio_reset_pin(config.trig_pin);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to reset the trigger GPIO Pin [%d].", config.trig_pin);
        return;
    }

    err = gpio_reset_pin(config.echo_pin);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to reset the echo GPIO Pin [%d].", config.echo_pin);
        return;
    }

    err = gpio_set_direction(config.trig_pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure the trigger GPIO Pin [%d] as output.", config.trig_pin);
        return;
    }

    err = gpio_set_direction(config.echo_pin, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure the echo GPIO Pin [%d] as input.", config.echo_pin);
        return;
    }

    // Configure the TRIG pin.
    gpio_config_t trigger_config = {};
    trigger_config.pin_bit_mask = (1ULL << config.trig_pin);    // Set the pin's bit mask.
    trigger_config.mode = GPIO_MODE_OUTPUT;                     // Set the pin to be an output.
    trigger_config.pull_up_en = GPIO_PULLUP_DISABLE;            // Disable the pull-up resistor.
    trigger_config.pull_down_en = GPIO_PULLDOWN_DISABLE;        // Disable the pull-down resistor.
    trigger_config.intr_type = GPIO_INTR_DISABLE;               // Disable the interrupt for this pin.

    err = gpio_config(&trigger_config);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure ultrasonic sensor trigger GPIO.");
        return;
    }

    err = gpio_set_level(config.trig_pin, 0);    // Initially set the TRIG pin to low.
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to set the trigger GPIO Pin [%d] to low [0].", config.trig_pin);
        return;
    }

    // Configure the ECHO pin.
    gpio_config_t echo_config = {};
    echo_config.pin_bit_mask = (1ULL << config.echo_pin);   // Set the pin's bit mask.
    echo_config.mode = GPIO_MODE_INPUT;                     // Set the pin to be an input.
    echo_config.pull_up_en = GPIO_PULLUP_DISABLE;           // Disable the pull-up resistor.
    echo_config.pull_down_en = GPIO_PULLDOWN_DISABLE;       // Disable the pull-down resistor.
    echo_config.intr_type = GPIO_INTR_ANYEDGE;              // Set the interrupt to trigger on both falling and rising edges.

    err = gpio_config(&echo_config);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to configure ultrasonic sensor echo GPIO.");
        return;
    }

    err = gpio_isr_handler_add(config.echo_pin, isr_handler, this);
    if (err != ESP_OK) {
        ESP_LOGW(config.name.c_str(), "Failed to initialize ultrasonic sensor ISR.");
        return;
    }

    ESP_LOGI(
        config.name.c_str(),
        "Initialized ultrasonic sensor on trigger GPIO Pin [%d] and echo GPIO pin [%d], set trigger GPIO pin to low, and initialized ISR.",
        config.trig_pin,
        config.echo_pin
    );

    
    state = Ultrasonic_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the initialized boolean.
    ============================================================
*/
bool Ultrasonic_Sensor::is_initialized() const {
    return state == Ultrasonic_State::READY;
}
//  ============================================================


/*
    ============================================================
    Retrieves the faulted boolean.
    ============================================================
*/
bool Ultrasonic_Sensor::is_faulted() const {
    return state == Ultrasonic_State::FAULT;
}
//  ============================================================


/*
    ============================================================
    Interrupt Service Routine that performs distance measurement
    through recording ECHO signal duration.
    ============================================================
*/
void IRAM_ATTR Ultrasonic_Sensor::isr_handler(void *arg) {
    auto *us = static_cast<Ultrasonic_Sensor*>(arg);

    // Return if the distance measurement is not active.
    if (!us->measurement_active.load(std::memory_order_acquire)) {
        return;
    }


    // Start ECHO signal timer.
    int64_t now_us = static_cast<int64_t>(esp_timer_get_time());
    
    // Record the start time and return if the ECHO pin is HIGH.
    int level = gpio_get_level(us->config.echo_pin);
    if (level == 1) {
        us->echo_start_time_us.store(now_us, std::memory_order_relaxed);
        return;
    }

    // Calculate the ECHO signal duration.
    int64_t start_us = us->echo_start_time_us.load(std::memory_order_relaxed);
    if (start_us == 0) {
        return;
    }

    int64_t duration_us = now_us - start_us;
    us->echo_duration_us.store(duration_us, std::memory_order_release);
    us->echo_start_time_us.store(0, std::memory_order_relaxed);
    us->measurement_active.store(false, std::memory_order_release);

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
bool Ultrasonic_Sensor::measure_distance() {
    if (state == Ultrasonic_State::UNINITIALIZED) {
        ESP_LOGW(config.name.c_str(), "Ultrasonic sensor is not initialized. Ignoring measure_distance().");
        return false;
    }
    else if (state == Ultrasonic_State::FAULT) {
        ESP_LOGW(config.name.c_str(), "Ultrasonic sensor is in fault state. Ignoring measure_distance().");
        return false;
    }

    // Reset variables that perform distance measurement.
    bool measurement_reset = reset_measurement();
    if (measurement_reset == false) {
        return false;
    }


    // Trigger the ultrasonic sensor.
    bool is_triggered = trigger_sensor();
    if (is_triggered == false) {
        return false;
    }


    // Wait for the ISR to finish measuring the ECHO signal duration.
    bool echo_returned = wait_for_echo();
    if (echo_returned == false) {
        return false;
    }


    // Measure distance of the object from the robot.
    int64_t duration_us = echo_duration_us.load(std::memory_order_acquire);
    double distance_calculated = calculate_distance(duration_us);
    if (distance_calculated == -1.0) {
        return false;
    }

    return true;
}
//  ============================================================


/*
    ============================================================
    Reset the variables used to perform distance measurement.
    ============================================================
*/
bool Ultrasonic_Sensor::reset_measurement() {
    if (measurement_active.load(std::memory_order_acquire)) {
        ESP_LOGE(config.name.c_str(), "Distance measurement is already in progress.");
        return false;
    }

    echo_start_time_us.store(0, std::memory_order_relaxed);
    echo_duration_us.store(0, std::memory_order_relaxed);
    distance_mm = -1.0;
    measurement_active.store(true, std::memory_order_release);

    return true;
}
//  ============================================================


/*
    ============================================================
    Trigger a valid pulse from the ultrasonic sensor for 10
    microseconds.
    ============================================================
*/
bool Ultrasonic_Sensor::trigger_sensor() {
    esp_err_t err = gpio_set_level(config.trig_pin, 1);
    if (err != ESP_OK) {
        measurement_active.store(false, std::memory_order_release);
        enter_fault("Trigger Pin: gpio_set_level(HIGH)", err);
        return false;
    }

    esp_rom_delay_us(10);

    err = gpio_set_level(config.trig_pin, 0);
    if (err != ESP_OK) {
        measurement_active.store(false, std::memory_order_release);
        enter_fault("Trigger Pin: gpio_set_level(LOW)", err);
        return false;
    }

    return true;
}
//  ============================================================


/*
    ============================================================
    Wait for the ECHO signal to return.
    ============================================================
*/
bool Ultrasonic_Sensor::wait_for_echo() {
    TickType_t start_tick = xTaskGetTickCount();
    while (measurement_active.load(std::memory_order_acquire)) {
        // Return if ECHO signal was lost.
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MEASUREMENT_TIMEOUT_MS)) {
            measurement_active.store(false, std::memory_order_release);
            distance_mm = -1.0;
            ESP_LOGI(config.name.c_str(), "Signal timed out by either being lost or went out of range.");
            return false;
        }

        vTaskDelay(1);
    }

    return true;
}
//  ============================================================


/*
    ============================================================
    Calculate the object's distance from the robot.
    ============================================================
*/
double Ultrasonic_Sensor::calculate_distance(int64_t duration_us) {
    if (duration_us == 0) {
        distance_mm = -1.0;
        ESP_LOGI(config.name.c_str(), "Signal timed out by either being lost or went out of range. Resetting distance.");
        return distance_mm;
    }

    distance_mm = (static_cast<double>(duration_us) * SPEED_OF_SOUND_MM_PER_US) / 2.0;
    ESP_LOGI(config.name.c_str(), "Object detected at [%0.4fmm].", distance_mm);

    return distance_mm;
}
//  ============================================================


/*
    ============================================================
    Retrieve calculated distance.
    ============================================================
*/
double Ultrasonic_Sensor::get_distance() const {
    return distance_mm;
}
//  ============================================================


/*
    ============================================================
    Retrieve echo's start time.
    ============================================================
*/
int64_t Ultrasonic_Sensor::get_echo_start_time() const{
    return echo_start_time_us;
}
//  ============================================================


/*
    ============================================================
    Record echo's start time.
    ============================================================
*/
void Ultrasonic_Sensor::set_echo_start_time(int64_t time) {
    echo_start_time_us = time;
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve echo pin.
    ============================================================
*/
gpio_num_t Ultrasonic_Sensor::get_echo_pin() {
    return config.echo_pin;
}
//  ============================================================


/*
    ============================================================
    Retrieve trig pin.
    ============================================================
*/
gpio_num_t Ultrasonic_Sensor::get_trig_pin() {
    return config.trig_pin;
}
//  ============================================================


/*
    ============================================================
    Enters the ultrasonic sensor into a fault state if an error
    occurs after initialization.
    ============================================================
*/
void Ultrasonic_Sensor::enter_fault(const char* operation, esp_err_t err) {
    ESP_LOGE(
        config.name.c_str(),
        "Ultrasonic sensor hardware failure during: [%s]. ESP error: [%s]. Entering fault state.",
        operation,
        esp_err_to_name(err)
    );

    state = Ultrasonic_State::FAULT;
    distance_mm = -1.0;
    measurement_active.store(false, std::memory_order_release);

    // Set TRIG pin to LOW as best effort.
    gpio_set_level(config.trig_pin, 0);

    return;
}
//  ============================================================