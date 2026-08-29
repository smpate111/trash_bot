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

#include <sensors/wheel_encoder.hpp>

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



/*
    ============================================================
    Test the wheel encoder class and its functions.
    ============================================================
*/
void test_wheel_encoder_functions(void) {
    // Create the wheel encoder object.
    Encoder_Config config = {
        "Encoder", GPIO_NUM_1,
        80.0, 20
    };

    Wheel_Encoder encoder(config);


    /*
        ============================================================
        Test 1: Verify the hardware initialization of the wheel
        encoder.
        ============================================================
    */

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_reset_pin_fake.arg0_history[0]);

    // Confirm the GPIO pins' directions.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[0]);

    // Confirm the number of calls for the GPIO config.
    TEST_ASSERT_EQUAL(1, gpio_config_fake.call_count);

    // Confirm the number of calls for the ISR handler and which GPIO pin called it.
    TEST_ASSERT_EQUAL(1, gpio_isr_handler_add_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_isr_handler_add_fake.arg0_history[0]);
    //  ============================================================


    /*
        ============================================================
        Test 2: Verify the Interrupt Service Routine gets triggered
        and performs pulse counting.
        ============================================================
    */

    // Verify the pulse count is 0 before performing the count.
    TEST_ASSERT_EQUAL(0, encoder.pulse_count);

    // Perform the count 5 times by calling the ISR handler 5 times.
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);
    Wheel_Encoder::isr_handler(&encoder);

    // Confirm the pulse count went up to 5.
    TEST_ASSERT_EQUAL(5, encoder.pulse_count);

    //  ============================================================


    /*
        ============================================================
        Test 3: Check if the calculate_distance() function math
        performs correct measurements.
        ============================================================
    */
    // Set the pulse count to 20 to mimic 1 full rotation.
    encoder.pulse_count = 20;

    // Calculate the distance traveled using the same equation in calculate_distance().
    double calculated_distance = encoder.pulse_count * ((3.142857 * config.wheel_diameter) / config.encoder_slots);

    // Confirm our calculated answer is the same as the answer received from calculate_distance().
    TEST_ASSERT_FLOAT_WITHIN(0.001, calculated_distance, encoder.calculate_distance());
    //  ============================================================


    /*
        ============================================================
        Test 4: Check if the reset_count() function resets the
        pulse count.
        ============================================================
    */
    // Confirm that the pulse count resets to 0.
    encoder.reset_count();
    TEST_ASSERT_EQUAL(0, encoder.pulse_count);
    //  ============================================================

    return;
}