/*
    This file tests the project's logic by simulating the ultrasonic
    sensor hardware when going through the CI/CD pipeline due to
    GitHub not being able to test the actual hardware.
*/


/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include "sensors/ultrasonic.hpp"

#include "mock_libraries/mock_hardware.hpp"

//  ============================================================



// Create a pointer that simulates the ultrasonic sensor.
static Ultrasonic_Sensor *ultrasonic_ptr = nullptr;

// Fake gpio_set_level() function to simulate ultrasonic ISR handler.
esp_err_t simulate_ultrasonic_echo(gpio_num_t gpio_num, uint32_t level) {
    if (ultrasonic_ptr == nullptr) {
        return ESP_OK;
    }

    // Simulate ECHO signal behavior when the TRIG pin is being controlled.
    if (gpio_num != GPIO_NUM_2) {
        return ESP_OK;
    }

    // If TRIG pin is set to HIGH, then simulate the ECHO pin to rising edge.
    if (level == 1) {
        gpio_get_level_fake.return_val = 1;
        Ultrasonic_Sensor::isr_handler(ultrasonic_ptr);
    }
    else {
        gpio_get_level_fake.return_val = 0;
        Ultrasonic_Sensor::isr_handler(ultrasonic_ptr);
    }

    return ESP_OK;
}



/*
    ============================================================
    Test that the Ultrasonic's constructor initializes
    correctly.
    ============================================================
*/
void test_ultrasonic_hardware_initialization(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_reset_pin_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_42, gpio_reset_pin_fake.arg0_history[1]);

    // Verify the TRIG GPIO pin's direction is correctly set to OUTPUT.
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[0]);

    // Verify the ECHO GPIO pin's direction is correctly set to INPUT.
    TEST_ASSERT_EQUAL(GPIO_NUM_42, gpio_set_direction_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[1]);

    // Verify the configurations for the TRIG GPIO config is correctly passed.
    TEST_ASSERT_EQUAL(2, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(1ULL << GPIO_NUM_2, gpio_config_fake.arg0_history[0]->pin_bit_mask);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_config_fake.arg0_history[0]->mode);
    TEST_ASSERT_EQUAL(GPIO_PULLUP_DISABLE, gpio_config_fake.arg0_history[0]->pull_up_en);
    TEST_ASSERT_EQUAL(GPIO_PULLDOWN_DISABLE, gpio_config_fake.arg0_history[0]->pull_down_en);
    TEST_ASSERT_EQUAL(GPIO_INTR_DISABLE, gpio_config_fake.arg0_history[0]->intr_type);

    // Verify the configurations for the ECHO GPIO config is correctly passed.
    TEST_ASSERT_EQUAL(2, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(1ULL << GPIO_NUM_42, gpio_config_fake.arg0_history[1]->pin_bit_mask);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_config_fake.arg0_history[1]->mode);
    TEST_ASSERT_EQUAL(GPIO_PULLUP_DISABLE, gpio_config_fake.arg0_history[1]->pull_up_en);
    TEST_ASSERT_EQUAL(GPIO_PULLDOWN_DISABLE, gpio_config_fake.arg0_history[1]->pull_down_en);
    TEST_ASSERT_EQUAL(GPIO_INTR_ANYEDGE, gpio_config_fake.arg0_history[1]->intr_type);

    // Verify that the ISR is correctly configured.
    TEST_ASSERT_EQUAL(1, gpio_isr_handler_add_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_42, gpio_isr_handler_add_fake.arg0_history[0]);

    TEST_ASSERT_TRUE(ultrasonic.is_initialized());
    TEST_ASSERT_FALSE(ultrasonic.is_faulted());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic's constructor fails initialization
    when there is a GPIO reset failure.
    ============================================================
*/
void test_ultrasonic_initialization_gpio_reset_failure(void) {
    // Configure the fake GPIO reset function to fail.
    gpio_reset_pin_fake.return_val = ESP_FAIL;

    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the reset was called.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(ultrasonic.is_initialized());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic's constructor fails initialization
    when there is a GPIO direction failure.
    ============================================================
*/
void test_ultrasonic_initialization_gpio_direction_failure(void) {
    // Configure the fake GPIO direction function to fail.
    gpio_set_direction_fake.return_val = ESP_FAIL;

    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(ultrasonic.is_initialized());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic's constructor fails initialization
    when GPIO pins are the same.
    ============================================================
*/
void test_ultrasonic_initialization_identical_GPIO_pins(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_2};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the GPIO pins were not reset.
    TEST_ASSERT_EQUAL(0, gpio_reset_pin_fake.call_count);

    // Confirm the direction was not called.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(ultrasonic.is_initialized());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic's constructor fails initialization
    when there is a GPIO config failure.
    ============================================================
*/
void test_ultrasonic_initialization_gpio_config_failure(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the GPIO pins were not reset.
    TEST_ASSERT_EQUAL(0, gpio_reset_pin_fake.call_count);

    // Confirm the direction was not called.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(ultrasonic.is_initialized());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic's constructor fails initialization
    when there is an ISR failure.
    ============================================================
*/
void test_ultrasonic_initialization_isr_failure(void) {
    // Configure the fake ISR function to fail.
    gpio_isr_handler_add_fake.return_val = ESP_FAIL;
    
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);

    // Confirm the config was called.
    TEST_ASSERT_EQUAL(2, gpio_config_fake.call_count);

    // Confirm the ISR was called.
    TEST_ASSERT_EQUAL(1, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(ultrasonic.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic's lockout works when initialization
    failed.
    ============================================================
*/
void test_ultrasonic_commands_lockout_after_initialization_failure(void) {
    // Configure the fake GPIO reset function to fail.
    gpio_reset_pin_fake.return_val = ESP_FAIL;

    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    // Confirm the reset was called.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(ultrasonic.is_initialized());

    ultrasonic.measure_distance();

    // Verify that no sensor commands have reached the object.
    TEST_ASSERT_EQUAL(-1.0, ultrasonic.get_distance());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic calculates the correct distance
    from a known ECHO signal duration.
    ============================================================
*/
void test_ultrasonic_distance_calculation(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    TEST_ASSERT_TRUE(ultrasonic.is_initialized());

    // Assume that the ECHO signal duration is 1000 microseconds.
    // This means that the distance is: (1000 us * 0.343 mm/us) / 2 = 171.5 mm.
    double calculated_distance = ultrasonic.calculate_distance(1000);

    // Verify the calculated distance is correct and the recorded distance is correct.
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 171.5, calculated_distance);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 171.5, ultrasonic.get_distance());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic performs distance measurements
    correctly.
    ============================================================
*/
void test_ultrasonic_measure_distance(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    TEST_ASSERT_TRUE(ultrasonic.is_initialized());

    // Configure the ultrasonic pointer and fake gpio_set_level() function.
    ultrasonic_ptr = &ultrasonic;
    gpio_set_level_fake.custom_fake = simulate_ultrasonic_echo;

    // Simulate ECHO signal times.
    int64_t echo_times[2] = {10000, 20000};
    SET_RETURN_SEQ(esp_timer_get_time, echo_times, 2);

    // Measure the ECHO signal distance.
    xTaskGetTickCount_fake.return_val = 0;
    bool measurement_success = ultrasonic.measure_distance();

    // Verify the measurement was successful.
    TEST_ASSERT_TRUE(measurement_success);

    // Verify the gpio_set_level calls.
    TEST_ASSERT_EQUAL(3, gpio_set_level_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_set_level_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(0, gpio_set_level_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_set_level_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(1, gpio_set_level_fake.arg1_history[1]);

    // Verify the esp_timer_get_time call.
    TEST_ASSERT_EQUAL(2, esp_timer_get_time_fake.call_count);

    // Verify the gpio_get_level call.
    TEST_ASSERT_EQUAL(2, gpio_get_level_fake.call_count);

    // Verify that the calculated distance is correct.
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 1715.0, ultrasonic.get_distance());

    // Clean up the fake functions.
    ultrasonic_ptr = nullptr;
    gpio_set_level_fake.custom_fake = nullptr;

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic performs timeouts when no ECHO
    signal is received before the timeout threshold is reached.
    ============================================================
*/
void test_ultrasonic_measure_distance_timeout(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    TEST_ASSERT_TRUE(ultrasonic.is_initialized());

    // Simulate the ECHO signal times.
    gpio_set_level_fake.return_val = ESP_OK;
    TickType_t ticks[2] = {
        100,
        100 + pdMS_TO_TICKS(Ultrasonic_Sensor::MEASUREMENT_TIMEOUT_MS)
    };
    SET_RETURN_SEQ(xTaskGetTickCount, ticks, 2);
    vTaskDelay_fake.arg0_val = 0;

    // Verify the measurement timed out.
    bool measurement_success = ultrasonic.measure_distance();
    TEST_ASSERT_FALSE(measurement_success);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -1.0, ultrasonic.get_distance());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Ultrasonic ISR ignores an ECHO signal when a
    distance measurement has not started.
    ============================================================
*/
void test_ultrasonic_isr_ignores_echo_signal(void) {
    // Create the ultrasonic object.
    Ultrasonic_Config config = {"Ultrasonic", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Sensor ultrasonic(config);

    TEST_ASSERT_TRUE(ultrasonic.is_initialized());

    // Simulate the ECHO signal that triggers the ISR.
    gpio_get_level_fake.return_val = 1;
    Ultrasonic_Sensor::isr_handler(&ultrasonic);

    // Verify that the ISR ignores the ECHO signal.
    TEST_ASSERT_EQUAL(0, esp_timer_get_time_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_get_level_fake.call_count);

    return;
}
//  ============================================================