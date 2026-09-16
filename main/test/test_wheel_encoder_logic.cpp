/*
    This file tests the project's logic by simulating the wheel encoder
    hardware when going through the CI/CD pipeline due to GitHub not
    being able to test the actual hardware.
*/


/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include "sensors/wheel_encoder.hpp"

#include "mock_libraries/mock_hardware.hpp"

//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor initializes
    correctly.
    ============================================================
*/
void test_wheel_encoder_hardware_initialization(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_reset_pin_fake.arg0_history[0]);

    // Verify the GPIO pin's direction is correctly set to INPUT.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[0]);

    // Verify the configurations for the GPIO config is correctly passed.
    TEST_ASSERT_EQUAL(1, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(1ULL << GPIO_NUM_1, gpio_config_fake.arg0_history[0]->pin_bit_mask);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_config_fake.arg0_history[0]->mode);
    TEST_ASSERT_EQUAL(GPIO_PULLUP_ENABLE, gpio_config_fake.arg0_history[0]->pull_up_en);
    TEST_ASSERT_EQUAL(GPIO_PULLDOWN_DISABLE, gpio_config_fake.arg0_history[0]->pull_down_en);
    TEST_ASSERT_EQUAL(GPIO_INTR_NEGEDGE, gpio_config_fake.arg0_history[0]->intr_type);

    // Verify that the ISR is correctly configured.
    TEST_ASSERT_EQUAL(1, gpio_isr_handler_add_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_isr_handler_add_fake.arg0_history[0]);

    TEST_ASSERT_TRUE(encoder.is_initialized());
    TEST_ASSERT_FALSE(encoder.is_faulted());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor fails
    initialization when there is a GPIO reset failure.
    ============================================================
*/
void test_wheel_encoder_initialization_gpio_reset_failure(void) {
    // Configure the fake GPIO reset function to fail.
    gpio_reset_pin_fake.return_val = ESP_FAIL;

    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the reset was called.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor fails
    initialization when there is a GPIO direction failure.
    ============================================================
*/
void test_wheel_encoder_initialization_gpio_direction_failure(void) {
    // Configure the fake GPIO direction function to fail.
    gpio_set_direction_fake.return_val = ESP_FAIL;
    
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor fails
    initialization when there are invalid encoder slots.
    ============================================================
*/
void test_wheel_encoder_initialization_invalid_slots(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, -1};
    Wheel_Encoder encoder(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor fails
    initialization when there is invalid wheel diameter.
    ============================================================
*/
void test_wheel_encoder_initialization_invalid_diameter(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor fails
    initialization when there is a GPIO config failure.
    ============================================================
*/
void test_wheel_encoder_initialization_gpio_config_failure(void) {
    // Configure the fake GPIO config function to fail.
    gpio_config_fake.return_val = ESP_FAIL;
    
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm the config was called.
    TEST_ASSERT_EQUAL(1, gpio_config_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's constructor fails
    initialization when there is an ISR failure.
    ============================================================
*/
void test_wheel_encoder_initialization_isr_failure(void) {
    // Configure the fake ISR function to fail.
    gpio_isr_handler_add_fake.return_val = ESP_FAIL;
    
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm the direction was called.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm the config was called.
    TEST_ASSERT_EQUAL(1, gpio_config_fake.call_count);

    // Confirm the ISR was called.
    TEST_ASSERT_EQUAL(1, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's lockout works when
    initialization failed.
    ============================================================
*/
void test_wheel_encoder_commands_lockout_after_initialization(void) {
    // Configure the fake GPIO reset function to fail.
    gpio_reset_pin_fake.return_val = ESP_FAIL;

    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    // Confirm the reset was called.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_config_fake.call_count);
    TEST_ASSERT_EQUAL(0, gpio_isr_handler_add_fake.call_count);

    TEST_ASSERT_FALSE(encoder.is_initialized());

    encoder.set_pulse_count(10);
    encoder.reset_count();
    encoder.calculate_distance();

    // Verify that no sensor commands have reached the object.
    TEST_ASSERT_EQUAL(0, encoder.get_pulse_count());
    TEST_ASSERT_EQUAL(0, encoder.get_pulse_count() * ((encoder.PI * config.wheel_diameter) / config.encoder_slots));

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's initial pulse count is 0, 
    set_pulse_count() updates the count, get_pulse_count()
    retrieves the count, and reset_count() resets the count.
    ============================================================
*/
void test_wheel_encoder_pulse_count(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    TEST_ASSERT_TRUE(encoder.is_initialized());

    // Confirm the initial pulse count is 0 and get_pulse_count() works.
    TEST_ASSERT_EQUAL(0, encoder.get_pulse_count());

    // Confirm the set_pulse_count() works.
    encoder.set_pulse_count(10);
    TEST_ASSERT_EQUAL(10, encoder.get_pulse_count());

    // Confirm the reset_count() works.
    encoder.reset_count();
    TEST_ASSERT_EQUAL(0, encoder.get_pulse_count());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's ISR gets triggered and
    increases the pulse count.
    ============================================================
*/
void test_wheel_encoder_trigger_ISR(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    TEST_ASSERT_TRUE(encoder.is_initialized());

    // Confirm the initial pulse count is 0.
    TEST_ASSERT_EQUAL(0, encoder.get_pulse_count());

    // Trigger ISR once.
    Wheel_Encoder::isr_handler(&encoder);

    // Confirm the pulse count went up to 1.
    TEST_ASSERT_EQUAL(1, encoder.get_pulse_count());

    // Trigger ISR 4 more times.
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);

    // Confirm the pulse count went up to 5.
    TEST_ASSERT_EQUAL(5, encoder.get_pulse_count());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Wheel Encoder's distance calculations for
    different revolutions.
    ============================================================
*/
void test_wheel_encoder_distance_calculations(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {"Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Wheel_Encoder encoder(config);

    TEST_ASSERT_TRUE(encoder.is_initialized());

    // Confirm the initial pulse count is 0.
    TEST_ASSERT_EQUAL(0, encoder.get_pulse_count());
    double calculated_distance = encoder.get_pulse_count() * ((encoder.PI * config.wheel_diameter) / config.encoder_slots);
    TEST_ASSERT_EQUAL(calculated_distance, encoder.calculate_distance());

    // Confirm the encoder calculates the distance for 1 full revolution.
    encoder.set_pulse_count(20);
    TEST_ASSERT_EQUAL(20, encoder.get_pulse_count());
    calculated_distance = encoder.get_pulse_count() * ((encoder.PI * config.wheel_diameter) / config.encoder_slots);
    TEST_ASSERT_EQUAL(calculated_distance, encoder.calculate_distance());

    // Confirm the encoder calculates the distance for 4 full revolutions.
    encoder.set_pulse_count(80);
    TEST_ASSERT_EQUAL(80, encoder.get_pulse_count());
    calculated_distance = encoder.get_pulse_count() * ((encoder.PI * config.wheel_diameter) / config.encoder_slots);
    TEST_ASSERT_EQUAL(calculated_distance, encoder.calculate_distance());

    // Confirm the encoder calculates the distance for 0.5 revolutions.
    encoder.set_pulse_count(10);
    TEST_ASSERT_EQUAL(10, encoder.get_pulse_count());
    calculated_distance = encoder.get_pulse_count() * ((encoder.PI * config.wheel_diameter) / config.encoder_slots);
    TEST_ASSERT_EQUAL(calculated_distance, encoder.calculate_distance());

    return;
}
//  ============================================================