/*
    This file tests the project's logic by simulating the motor driver
    hardware when going through the CI/CD pipeline due to GitHub not
    being able to test the actual hardware.
*/


/*
    ============================================================
    Define the file's dependencies.
    ============================================================
*/

#include <motors/motor.hpp>
#include <motors/motor_driver.hpp>

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



/*
    ============================================================
    Test the motor driver class and its functions.
    ============================================================
*/
void test_motor_driver_functions(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver object.
    Driver_Config config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(config);


    /*
        ============================================================
        Test 1: Verify the hardware initialization of the motor
        driver.
        ============================================================
    */

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(4, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_reset_pin_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_reset_pin_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_NUM_13, gpio_reset_pin_fake.arg0_history[2]);
    TEST_ASSERT_EQUAL(GPIO_NUM_14, gpio_reset_pin_fake.arg0_history[3]);

    // Confirm the GPIO pins' directions.
    TEST_ASSERT_EQUAL(4, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_set_direction_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(GPIO_NUM_13, gpio_set_direction_fake.arg0_history[2]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(GPIO_NUM_14, gpio_set_direction_fake.arg0_history[3]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[3]);

    // Confirm the number of calls for the LEDC config.
    TEST_ASSERT_EQUAL(4, ledc_channel_config_fake.call_count);
    //  ============================================================
    

    /*
        ============================================================
        Test 2: Check if the fake LEDC hardware is correctly called
        in forward().
        ============================================================
    */
    driver.adjust_speed(255, 255);
    driver.forward();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[0]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[0]);       // config.Left_Motor.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[0]);     // Current_Speed

    // Confirm the LEDC duty for the left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[1]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[1]);       // config.Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[1]);     // 0.0

    // Confirm the LEDC duty for the right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[2]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[2]);       // config.Right_Motor.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[2]);     // Current_Speed

    // Confirm the LEDC duty for the right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[3]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[3]);       // config.Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[3]);     // 0.0
    //  ============================================================


    /*
        ============================================================
        Test 3: Check if the fake LEDC hardware is correctly called
        in backward().
        ============================================================
    */
    driver.adjust_speed(200, 200);
    driver.backward();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[4]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[4]);       // config.Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[4]);     // 0.0

    // Confirm the LEDC duty for the left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[5]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[5]);       // config.Left_Motor.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[5]);     // Current_Speed

    // Confirm the LEDC duty for the right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[6]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[6]);       // config.Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[6]);     // 0.0

    // Confirm the LEDC duty for the right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[7]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[7]);       // config.Right_Motor.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[7]);     // Current_Speed
    //  ============================================================


    /*
        ============================================================
        Test 4: Check if the fake LEDC hardware is correctly called
        in left().
        ============================================================
    */
    driver.adjust_speed(150, 150);
    driver.left();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(12, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[8]);        // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[8]);        // config.Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[8]);      // 0.0

    // Confirm the LEDC duty for the left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[9]);        // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[9]);        // config.Left_Motor.channel_2
    TEST_ASSERT_EQUAL(150, ledc_set_duty_fake.arg2_history[9]);      // Current_Speed

    // Confirm the LEDC duty for the right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[10]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[10]);       // config.Right_Motor.channel_1
    TEST_ASSERT_EQUAL(150, ledc_set_duty_fake.arg2_history[10]);     // Current_Speed

    // Confirm the LEDC duty for the right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[11]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[11]);       // config.Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[11]);     // 0.0
    //  ============================================================


    /*
        ============================================================
        Test 5: Check if the fake LEDC hardware is correctly called
        in right().
        ============================================================
    */
    driver.adjust_speed(100, 100);
    driver.right();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(16, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[12]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[12]);       // config.Left_Motor.channel_1
    TEST_ASSERT_EQUAL(100, ledc_set_duty_fake.arg2_history[12]);     // Current_Speed

    // Confirm the LEDC duty for the left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[13]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[13]);       // config.Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[13]);     // 0.0

    // Confirm the LEDC duty for the right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[14]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[14]);       // config.Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[14]);     // 0.0

    // Confirm the LEDC duty for the right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[15]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[15]);       // config.Right_Motor.channel_2
    TEST_ASSERT_EQUAL(100, ledc_set_duty_fake.arg2_history[15]);     // Current_Speed
    //  ============================================================


    /*
        ============================================================
        Test 6: Check if the fake LEDC hardware is correctly called
        in stop().
        ============================================================
    */
    driver.brake();

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(20, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[16]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[16]);       // config.Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[16]);     // 0.0

    // Confirm the LEDC duty for the left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[17]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[17]);       // config.Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[17]);     // 0.0

    // Confirm the LEDC duty for the right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[18]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[18]);       // config.Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[18]);     // 0.0

    // Confirm the LEDC duty for the right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[19]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[19]);       // config.Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[19]);     // 0.0
    //  ============================================================

    return;
}