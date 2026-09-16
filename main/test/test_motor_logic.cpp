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

#include "motors/motor.hpp"

#include "mock_libraries/mock_hardware.hpp"

//  ============================================================



/*
    ============================================================
    Test that the Motor's constructor initializes correctly.
    ============================================================
*/
void test_motor_hardware_initialization(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_reset_pin_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_reset_pin_fake.arg0_history[1]);

    // Verify the GPIO pins' directions are correctly set to OUTPUT.
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_set_direction_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[1]);

    // Confirm the number of calls for the LEDC channel configuration.
    TEST_ASSERT_EQUAL(2, ledc_channel_config_fake.call_count);

    // Verify the configurations for both LEDC channels are correctly passed.
    TEST_ASSERT_EQUAL(GPIO_NUM_11, ledc_channel_config_fake.arg0_history[0]->gpio_num);
    TEST_ASSERT_EQUAL(LEDC_LOW_SPEED_MODE, ledc_channel_config_fake.arg0_history[0]->speed_mode);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_channel_config_fake.arg0_history[0]->channel);
    TEST_ASSERT_EQUAL(LEDC_TIMER_0, ledc_channel_config_fake.arg0_history[0]->timer_sel);
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.arg0_history[0]->duty);
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.arg0_history[0]->hpoint);

    TEST_ASSERT_EQUAL(GPIO_NUM_12, ledc_channel_config_fake.arg0_history[1]->gpio_num);
    TEST_ASSERT_EQUAL(LEDC_LOW_SPEED_MODE, ledc_channel_config_fake.arg0_history[1]->speed_mode);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_channel_config_fake.arg0_history[1]->channel);
    TEST_ASSERT_EQUAL(LEDC_TIMER_0, ledc_channel_config_fake.arg0_history[1]->timer_sel);
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.arg0_history[1]->duty);
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.arg0_history[1]->hpoint);

    TEST_ASSERT_TRUE(motor.is_initialized());
    TEST_ASSERT_FALSE(motor.is_faulted());

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());
    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor's constructor fails initialization when
    resetting the GPIO pin.
    ============================================================
*/
void test_motor_initialization_gpio_reset_failure(void) {
    // Configure the fake GPIO reset function to fail.
    gpio_reset_pin_fake.return_val = ESP_FAIL;

    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    // Confirm the initialization failed.
    TEST_ASSERT_FALSE(motor.is_initialized());

    // Verify that the constructor stopped after the first failed reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.call_count);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor's constructor fails initialization when
    setting the GPIO pin's direction.
    ============================================================
*/
void test_motor_initialization_gpio_direction_failure(void) {
    // Configure the fake GPIO direction function to fail.
    gpio_set_direction_fake.return_val = ESP_FAIL;

    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    // Confirm the initialization failed.
    TEST_ASSERT_FALSE(motor.is_initialized());

    // Verify that the constructor reset both pins.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);

    // Verify that the constructor stopped after the first failed direction set.
    TEST_ASSERT_EQUAL(1, gpio_set_direction_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.call_count);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor's constructor fails initialization when
    configuring the LEDC channel.
    ============================================================
*/
void test_motor_initialization_ledc_config_failure(void) {
    // Configure the fake LEDC channel to fail.
    ledc_channel_config_fake.return_val = ESP_FAIL;

    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    // Confirm the initialization failed.
    TEST_ASSERT_FALSE(motor.is_initialized());

    // Verify that the constructor reset both pins.
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);

    // Verify that the constructor set directions for both pins.
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);

    // Verify that the constructor stopped after the first failed LEDC channel configuration.
    TEST_ASSERT_EQUAL(1, ledc_channel_config_fake.call_count);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor commands are locked out when the
    initialization fails.
    ============================================================
*/
void test_motor_commands_lockout_after_initialization_failure(void) {
    // Configure the fake GPIO reset function to fail.
    gpio_reset_pin_fake.return_val = ESP_FAIL;

    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    // Confirm the initialization failed.
    TEST_ASSERT_FALSE(motor.is_initialized());

    // Verify that the constructor stopped after the first failed reset.
    TEST_ASSERT_EQUAL(1, gpio_reset_pin_fake.call_count);

    // Confirm that the rest of the initialization process didn't happen.
    TEST_ASSERT_EQUAL(0, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(0, ledc_channel_config_fake.call_count);

    motor.set_duty_cycle(100);
    motor.spin_forward();
    motor.spin_backward();
    motor.stop();

    // Verify that no actuator commands have reached the hardware.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(0, ledc_update_duty_fake.call_count);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor commands are locked out when runtime
    failure occurs.
    ============================================================
*/
void test_motor_enters_fault_after_ledc_set_failure(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    TEST_ASSERT_TRUE(motor.is_initialized());
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_channel_config_fake.call_count);

    motor.set_duty_cycle(100);

    ledc_set_duty_fake.return_val = ESP_FAIL;
    motor.spin_forward();

    TEST_ASSERT_TRUE(motor.is_faulted());
    TEST_ASSERT_FALSE(motor.is_initialized());
    TEST_ASSERT_EQUAL(0, motor.get_duty_cycle());

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor commands are locked out when runtime
    failure occurs.
    ============================================================
*/
void test_motor_enters_fault_after_ledc_update_failure(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    TEST_ASSERT_TRUE(motor.is_initialized());
    TEST_ASSERT_EQUAL(2, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(2, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_channel_config_fake.call_count);

    motor.set_duty_cycle(100);

    ledc_update_duty_fake.return_val = ESP_FAIL;
    motor.spin_backward();

    TEST_ASSERT_TRUE(motor.is_faulted());
    TEST_ASSERT_FALSE(motor.is_initialized());
    TEST_ASSERT_EQUAL(0, motor.get_duty_cycle());

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the set_duty_cycle() function correctly updates the
    motor's duty cycle to values in the 0-255 range.
    ============================================================
*/
void test_motor_set_duty_cycle(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);
    
    // Verify the duty cycle value gets updated.
    motor.set_duty_cycle(0);
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    motor.set_duty_cycle(1);
    TEST_ASSERT_EQUAL_UINT8(1, motor.get_duty_cycle());

    motor.set_duty_cycle(150);
    TEST_ASSERT_EQUAL_UINT8(150, motor.get_duty_cycle());

    motor.set_duty_cycle(254);
    TEST_ASSERT_EQUAL_UINT8(254, motor.get_duty_cycle());

    motor.set_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(255, motor.get_duty_cycle());
    return;
}
//  ============================================================



/*
    ============================================================
    Test the spin_forward() function correctly updates the PWM
    duty cycle for both LEDC channels.
    ============================================================
*/
void test_motor_forward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    motor.set_duty_cycle(255);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(255, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the spin_backward() function correctly updates the PWM
    duty cycle for both LEDC channels.
    ============================================================
*/
void test_motor_backward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    motor.set_duty_cycle(255);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(255, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the stop() function correctly updates the PWM duty
    cycle for both LEDC channels.
    ============================================================
*/
void test_motor_stop(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);

    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can spin forward and then forward.
    ============================================================
*/
void test_motor_forward_to_forward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Forward
    motor.set_duty_cycle(123);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(123, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());


    // Forward
    motor.set_duty_cycle(231);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(231, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can spin forward and then backward.
    ============================================================
*/
void test_motor_forward_to_backward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Forward
    motor.set_duty_cycle(231);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(231, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());


    // Backward
    motor.set_duty_cycle(123);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(123, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can spin forward and then stop.
    ============================================================
*/
void test_motor_forward_to_stop(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Forward
    motor.set_duty_cycle(100);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(100, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(100, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());


    // Stop
    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can spin backward and then forward.
    ============================================================
*/
void test_motor_backward_to_forward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Backward
    motor.set_duty_cycle(123);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(123, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());


    // Forward
    motor.set_duty_cycle(231);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(231, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can spin backward and then backward.
    ============================================================
*/
void test_motor_backward_to_backward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Backward
    motor.set_duty_cycle(231);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(231, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());


    // Backward
    motor.set_duty_cycle(123);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(123, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can spin backward and then stop.
    ============================================================
*/
void test_motor_backward_to_stop(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Backward
    motor.set_duty_cycle(100);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(100, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(100, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());


    // Stop
    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can stop spinning and then spin forward.
    ============================================================
*/
void test_motor_stop_to_forward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Stop
    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());


    // Forward
    motor.set_duty_cycle(231);
    motor.spin_forward();
    TEST_ASSERT_EQUAL_UINT8(231, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(231, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can stop spinning and then spin backward.
    ============================================================
*/
void test_motor_stop_to_backward(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Stop
    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());


    // Backward
    motor.set_duty_cycle(123);
    motor.spin_backward();
    TEST_ASSERT_EQUAL_UINT8(123, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(123, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, motor.get_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test the motor can stop spinning and then stop.
    ============================================================
*/
void test_motor_stop_to_stop(void) {
    // Create the motor object.
    Motor_Config config = {"Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor motor(config);


    // Stop
    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(2, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[0]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[1]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[1]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());


    // Stop
    motor.stop();
    TEST_ASSERT_EQUAL_UINT8(0, motor.get_duty_cycle());

    // Confirm the set duty and update duty were both called twice.
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(4, ledc_update_duty_fake.call_count);

    // Verify that the motor's channel 1 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_set_duty_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[2]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_0, ledc_update_duty_fake.arg1_history[2]);

    // Verify that the motor's channel 2 is correctly configured.
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_set_duty_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg2_history[3]);
    TEST_ASSERT_EQUAL(LEDC_CHANNEL_1, ledc_update_duty_fake.arg1_history[3]);

    TEST_ASSERT_EQUAL(Motor_Command::STOP, motor.get_motor_command());

    return;
}
//  ============================================================