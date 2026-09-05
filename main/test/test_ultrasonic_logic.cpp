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

#include <sensors/ultrasonic.hpp>

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



/*
    ============================================================
    Test the ultrasonic sensor class and its functions.
    ============================================================
*/
void test_ultrasonic_functions(void) {
    // Create the ultrasonic sensor object.
    Ultrasonic_Config config = {"Ultrasonic Sensor", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic ultrasonic(config);


    /*
        ============================================================
        Test 1: Verify the hardware initialization of the ultrasonic
        sensor.
        ============================================================
    */

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_reset_pin_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_42, gpio_reset_pin_fake.arg0_history[1]);

    // Confirm the GPIO pins' directions.
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_42, gpio_set_direction_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[1]);

    // Confirm the number of calls for the GPIO config.
    TEST_ASSERT_EQUAL(2, gpio_config_fake.call_count);

    // Confirm the number of calls for the ISR handler and which GPIO pin called it.
    TEST_ASSERT_EQUAL(1, gpio_isr_handler_add_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_42, gpio_isr_handler_add_fake.arg0_history[0]);
    //  ============================================================


    /*
        ============================================================
        Test 2: Verify the Interrupt Service Routine gets triggered
        and records a signal's start and end times.
        ============================================================
    */

    // Simulate time passing for 1 second (in microseconds).
    int64_t start_time = 10000;
    int64_t end_time = 20000;
    int64_t times[2] = {start_time, end_time};
    SET_RETURN_SEQ(esp_timer_get_time, times, 2);

    // Simulate ECHO pin going HIGH.
    gpio_get_level_fake.return_val = 1;
    Ultrasonic::isr_handler(&ultrasonic);

    // Simulate ECHO pin going LOW.
    gpio_get_level_fake.return_val = 0;
    Ultrasonic::isr_handler(&ultrasonic);

    // Confirm the number of calls for the timer and level.
    TEST_ASSERT_EQUAL(2, esp_timer_get_time_fake.call_count);
    TEST_ASSERT_EQUAL(2, gpio_get_level_fake.call_count);
    //  ============================================================


    /*
        ============================================================
        Test 3: Check if the measure_distance() function math
        performs correct measurements based on the signal length
        from the ISR.
        ============================================================
    */
    // Calculate the distance the signal traveled using the same equation in the ISR.
    double calculated_distance = ((end_time - start_time) * 0.343) / 2.0;

    // Confirm our calculated answer is the same as the answer received from the previous test.
    TEST_ASSERT_DOUBLE_WITHIN(0.001, calculated_distance, ultrasonic.get_distance());

    //  ============================================================

    return;
}