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

#include "motors/motor.hpp"
#include "motors/motor_driver.hpp"

#include "mock_libraries/mock_hardware.hpp"

//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor initializes
    correctly.
    ============================================================
*/
void test_motor_driver_hardware_initialization(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    TEST_ASSERT_TRUE(l_motor.is_initialized());
    TEST_ASSERT_TRUE(r_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);

    TEST_ASSERT_TRUE(driver.is_initialized());

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor fails
    initialization when initializing the Left Motor.
    ============================================================
*/
void test_motor_driver_initialization_left_motor_failure(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor r_motor(r_config);

    gpio_reset_pin_fake.return_val = ESP_FAIL;
    Motor l_motor(l_config);

    TEST_ASSERT_FALSE(l_motor.is_initialized());
    TEST_ASSERT_TRUE(r_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);

    TEST_ASSERT_FALSE(driver.is_initialized());

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor fails
    initialization when initializing the Right Motor.
    ============================================================
*/
void test_motor_driver_initialization_right_motor_failure(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);

    gpio_set_direction_fake.return_val = ESP_FAIL;
    Motor r_motor(r_config);

    TEST_ASSERT_TRUE(l_motor.is_initialized());
    TEST_ASSERT_FALSE(r_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);

    TEST_ASSERT_FALSE(driver.is_initialized());

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's commands are locked out when
    the initialization fails.
    ============================================================
*/
void test_motor_driver_commands_lockout_after_initialization(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);

    ledc_channel_config_fake.return_val = ESP_FAIL;
    Motor r_motor(r_config);

    TEST_ASSERT_TRUE(l_motor.is_initialized());
    TEST_ASSERT_FALSE(r_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);

    TEST_ASSERT_FALSE(driver.is_initialized());

    TEST_ASSERT_EQUAL(4, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(4, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(3, ledc_channel_config_fake.call_count);

    driver.set_left_duty_cycle(100);
    driver.set_right_duty_cycle(100);

    driver.forward();
    driver.backward();
    driver.left_turn();
    driver.right_turn();
    driver.stop();

    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(0, ledc_update_duty_fake.call_count);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's commands are locked out when
    runtime failure occurs.
    ============================================================
*/
void test_motor_driver_enters_fault_after_ledc_set_failure(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    TEST_ASSERT_TRUE(l_motor.is_initialized());
    TEST_ASSERT_TRUE(r_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);

    TEST_ASSERT_TRUE(driver.is_initialized());

    TEST_ASSERT_EQUAL(4, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(4, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_channel_config_fake.call_count);

    driver.set_left_duty_cycle(100);
    driver.set_right_duty_cycle(100);

    ledc_set_duty_fake.return_val = ESP_FAIL;
    driver.forward();

    TEST_ASSERT_TRUE(driver.is_faulted());
    TEST_ASSERT_FALSE(driver.is_initialized());
    TEST_ASSERT_EQUAL(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL(0, driver.get_right_duty_cycle());

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's commands are locked out when
    runtime failure occurs.
    ============================================================
*/
void test_motor_driver_enters_fault_after_ledc_update_failure(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    TEST_ASSERT_TRUE(l_motor.is_initialized());
    TEST_ASSERT_TRUE(r_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);

    TEST_ASSERT_TRUE(driver.is_initialized());

    TEST_ASSERT_EQUAL(4, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(4, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_channel_config_fake.call_count);

    driver.set_left_duty_cycle(100);
    driver.set_right_duty_cycle(100);

    ledc_update_duty_fake.return_val = ESP_FAIL;
    driver.backward();

    TEST_ASSERT_TRUE(driver.is_faulted());
    TEST_ASSERT_FALSE(driver.is_initialized());
    TEST_ASSERT_EQUAL(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL(0, driver.get_right_duty_cycle());

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the set_left_duty_cycle() and set_right_duty_cycle()
    functions correctly updates the motors' duty cycles to
    values in the 0-255 range.
    ============================================================
*/
void test_motor_driver_set_duty_cycles(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(0);
    driver.set_right_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_right_duty_cycle());

    driver.set_left_duty_cycle(1);
    driver.set_right_duty_cycle(254);
    TEST_ASSERT_EQUAL_UINT8(1, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(254, driver.get_right_duty_cycle());

    driver.set_left_duty_cycle(150);
    driver.set_right_duty_cycle(150);
    TEST_ASSERT_EQUAL_UINT8(150, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(150, driver.get_right_duty_cycle());

    driver.set_left_duty_cycle(254);
    driver.set_right_duty_cycle(1);
    TEST_ASSERT_EQUAL_UINT8(254, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(1, driver.get_right_duty_cycle());

    driver.set_left_duty_cycle(255);
    driver.set_right_duty_cycle(0);
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());
    return;
}
//  ============================================================



/*
    ============================================================
    Test the forward() function correctly spins both motors
    forward.
    ============================================================
*/
void test_motor_driver_forward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(255);
    driver.set_right_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the backward() function correctly spins both motors
    backward.
    ============================================================
*/
void test_motor_driver_backward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(255);
    driver.set_right_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the left_turn() function correctly spins the left motor
    backward and the right motor forward.
    ============================================================
*/
void test_motor_driver_left_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(255);
    driver.set_right_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the right_turn() function correctly spins the left
    motor forward and the right motor backward.
    ============================================================
*/
void test_motor_driver_right_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(255);
    driver.set_right_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(255, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the stop() function correctly stops the left motor and
    the right motor from spinning.
    ============================================================
*/
void test_motor_driver_stop(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move forward and then forward.
    ============================================================
*/
void test_motor_driver_forward_to_forward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move forward and then backward.
    ============================================================
*/
void test_motor_driver_forward_to_backward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move forward and then left turn.
    ============================================================
*/
void test_motor_driver_forward_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move forward and then right turn.
    ============================================================
*/
void test_motor_driver_forward_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move forward and then stop.
    ============================================================
*/
void test_motor_driver_forward_to_stop(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move backward and then forward.
    ============================================================
*/
void test_motor_driver_backward_to_forward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move backward and then backward.
    ============================================================
*/
void test_motor_driver_backward_to_backward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move backward and then left turn.
    ============================================================
*/
void test_motor_driver_backward_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move backward and then right turn.
    ============================================================
*/
void test_motor_driver_backward_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can move backward and then stop.
    ============================================================
*/
void test_motor_driver_backward_to_stop(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can left turn and then forward.
    ============================================================
*/
void test_motor_driver_left_turn_to_forward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can left turn and then backward.
    ============================================================
*/
void test_motor_driver_left_turn_to_backward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can left turn and then left turn.
    ============================================================
*/
void test_motor_driver_left_turn_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can left turn and then right turn.
    ============================================================
*/
void test_motor_driver_left_turn_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can left turn and then stop.
    ============================================================
*/
void test_motor_driver_left_turn_to_stop(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can right turn and then forward.
    ============================================================
*/
void test_motor_driver_right_turn_to_forward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can right turn and then backward.
    ============================================================
*/
void test_motor_driver_right_turn_to_backward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can right turn and then left turn.
    ============================================================
*/
void test_motor_driver_right_turn_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can right turn and then right turn.
    ============================================================
*/
void test_motor_driver_right_turn_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can right turn and then stop.
    ============================================================
*/
void test_motor_driver_right_turn_to_stop(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(123);
    driver.set_right_duty_cycle(123);
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can stop and then forward.
    ============================================================
*/
void test_motor_driver_stop_to_forward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.forward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can stop and then backward.
    ============================================================
*/
void test_motor_driver_stop_to_backward(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.backward();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can stop and then left turn.
    ============================================================
*/
void test_motor_driver_stop_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.left_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can stop and then right turn.
    ============================================================
*/
void test_motor_driver_stop_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.set_left_duty_cycle(231);
    driver.set_right_duty_cycle(231);
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, driver.get_right_duty_cycle());

    driver.right_turn();

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, driver.get_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor driver can stop and then stop.
    ============================================================
*/
void test_motor_driver_stop_to_stop(void) {
    // Create the motor objects.
    Motor_Config l_config = {"Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config r_config = {"Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver objects.
    Driver_Config d_config = {"Driver", l_motor, r_motor};
    Motor_Driver driver(d_config);
    
    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());


    // Verify the duty cycle value gets updated.
    driver.stop();
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, driver.get_right_duty_cycle());

    // Confirm the set duty and update duty were both called 4 times.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(8, ledc_update_duty_fake.call_count);

    // Verify that the left motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[4]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[4]);

    // Verify that the left motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[5]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[5]);

    // Verify that the right motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_set_duty_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[6]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_2, ledc_update_duty_fake.arg1_history[6]);

    // Verify that the right motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_set_duty_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[7]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_3, ledc_update_duty_fake.arg1_history[7]);

    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, driver.get_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, driver.get_right_motor_command());

    return;
}
//  ============================================================