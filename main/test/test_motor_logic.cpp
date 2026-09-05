/*
    This file tests the project's motor logic by simulating the motor hardware
    when going through the CI/CD pipeline due to GitHub not being able to test
    the actual hardware.
*/


/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <motors/motor.hpp>

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



/*
    ============================================================
    Test the motor class and its functions.
    ============================================================
*/
void test_motor_functions(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    /*
        ============================================================
        Test 1: Verify the hardware initialization of the motor.
        ============================================================
    */

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_reset_pin_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_reset_pin_fake.arg0_history[1]);

    // Confirm the GPIO pins' directions.
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_set_direction_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[1]);

    // Confirm the number of calls for the LEDC config.
    TEST_ASSERT_EQUAL(2, ledc_channel_config_fake.call_count);
    //  ============================================================

    
    /*
        ============================================================
        Test 2: Check if the fake LEDC hardware is correctly called
        in spin_forward().
        ============================================================
    */
    motor.set_speed(255);
    motor.spin_forward();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[0]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[0]);       // config.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[0]);     // Current_Speed

    // Confirm the LEDC duty for channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[1]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[1]);       // config.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[1]);     // 0.0
    //  ============================================================


    /*
        ============================================================
        Test 3: Check if the fake LEDC hardware is correctly called
        in spin_backward().
        ============================================================
    */
    motor.set_speed(200);
    motor.spin_backward();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[2]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[2]);       // config.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[2]);     // 0.0

    // Confirm the LEDC duty for channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[3]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[3]);       // config.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[3]);     // Current_Speed
    //  ============================================================


    /*
        ============================================================
        Test 4: Check if the fake LEDC hardware is correctly called
        in stop().
        ============================================================
    */
    motor.stop();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(6, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[4]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[4]);       // config.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[4]);     // 0.0

    // Confirm the LEDC duty for channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[5]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[5]);       // config.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[5]);     // 0.0
    //  ============================================================

    return;
}