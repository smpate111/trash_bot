/*
    This file tests the project's logic by simulating the drive train
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
#include "controllers/drive_train.hpp"

#include "mock_libraries/mock_hardware.hpp"

//  ============================================================



/*
    ============================================================
    Test that the Drive_Train's constructor initializes
    correctly.
    ============================================================
*/
void test_drive_train_hardware_initialization(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify the motors are initialized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify the motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify the wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train is initialized in the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor fails
    initialization when initializing the Front Motor Driver.
    ============================================================
*/
void test_drive_train_initialization_front_driver_failure(void) {
    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify the wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());

    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Set the front left motor to have an error.
    gpio_reset_pin_fake.return_val = ESP_FAIL;
    Motor fl_motor(fl_config);

    // Verify that all motors besides the front left motor are initialized.
    TEST_ASSERT_FALSE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers besides the front motor driver are initialized.
    TEST_ASSERT_FALSE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is not initialized.
    TEST_ASSERT_FALSE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train is initialized in the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor fails
    initialization when initializing the Rear Motor Driver.
    ============================================================
*/
void test_drive_train_initialization_rear_driver_failure(void) {
    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that the wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());

    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);

    // Set the rear right motor to have an error.
    gpio_set_direction_fake.return_val = ESP_FAIL;
    Motor rr_motor(rr_config);


    // Verify that all motors besides the rear right motor are initialized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_FALSE(rr_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers besides the rear motor driver are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_FALSE(rm_driver.is_initialized());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is not initialized.
    TEST_ASSERT_FALSE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train is initialized in the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor fails
    initialization when initializing the Left Wheel Encoder.
    ============================================================
*/
void test_drive_train_initialization_left_encoder_failure(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify the motors are initialized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify the motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder rw_encoder(rwe_config);

    // Set the left wheel encoder to have an error.
    gpio_reset_pin_fake.return_val = ESP_FAIL;
    Wheel_Encoder lw_encoder(lwe_config);

    // Verify that all wheel encoders besides the left wheel encoder are initialized.
    TEST_ASSERT_FALSE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is not initialized.
    TEST_ASSERT_FALSE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train is initialized in the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Motor_Driver's constructor fails
    initialization when initializing the Right Wheel Encoder.
    ============================================================
*/
void test_drive_train_initialization_right_encoder_failure(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify the motors are initialized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify the motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);

    // Set the right encoder to have an error.
    gpio_set_direction_fake.return_val = ESP_FAIL;
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders besides the right wheel encoder are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_FALSE(rw_encoder.is_initialized());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is not initialized.
    TEST_ASSERT_FALSE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train is initialized in the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Drive_Train's commands are locked out when the
    initialization fails.
    ============================================================
*/
void test_drive_train_commands_lockout_after_initialization(void) {
    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());

    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Set the front left motor to have an error.
    gpio_reset_pin_fake.return_val = ESP_FAIL;
    Motor fl_motor(fl_config);

    // Verify that all motors besides the front left motor are initialized.
    TEST_ASSERT_FALSE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers besides the front motor driver are intiialized.
    TEST_ASSERT_FALSE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is not initialized.
    TEST_ASSERT_FALSE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    TEST_ASSERT_EQUAL(9, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(8, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(6, ledc_channel_config_fake.call_count);

    // Verify the drive train commands are locked out.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(0, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, d_train.get_right_duty_cycle());

    d_train.forward();
    d_train.backward();
    d_train.left_turn();
    d_train.right_turn();
    d_train.stop();
    d_train.reset_encoder_counts();
    d_train.get_left_distance();
    d_train.get_right_distance();

    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(0, ledc_update_duty_fake.call_count);

    // Verify the drive train is set to the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the Drive_Train's commands are locked out when a
    failure occurs.
    ============================================================
*/
void test_drive_train_commands_lockout_after_failure(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify that the drive train commands are locked out when an error occurs.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_right_duty_cycle());

    ledc_set_duty_fake.return_val = ESP_FAIL;

    d_train.forward();
    d_train.backward();
    d_train.left_turn();
    d_train.right_turn();
    d_train.stop();
    d_train.reset_encoder_counts();

    TEST_ASSERT_TRUE(d_train.is_faulted());

    TEST_ASSERT_EQUAL(33, ledc_set_duty_fake.call_count);
    TEST_ASSERT_EQUAL(22, ledc_update_duty_fake.call_count);

    // Verify the drive train is set to the STOP state.
    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the left and right PWM duty cycles are
    configurable in drive train.
    ============================================================
*/
void test_drive_train_set_duty_cycles(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train's PWM duty cycles updated.
    d_train.set_left_duty_cycle(0);
    d_train.set_right_duty_cycle(255);
    TEST_ASSERT_EQUAL_UINT8(0, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(255, d_train.get_right_duty_cycle());

    d_train.set_left_duty_cycle(1);
    d_train.set_right_duty_cycle(254);
    TEST_ASSERT_EQUAL_UINT8(1, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(254, d_train.get_right_duty_cycle());

    d_train.set_left_duty_cycle(150);
    d_train.set_right_duty_cycle(150);
    TEST_ASSERT_EQUAL_UINT8(150, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(150, d_train.get_right_duty_cycle());

    d_train.set_left_duty_cycle(254);
    d_train.set_right_duty_cycle(1);
    TEST_ASSERT_EQUAL_UINT8(254, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(1, d_train.get_right_duty_cycle());

    d_train.set_left_duty_cycle(255);
    d_train.set_right_duty_cycle(0);
    TEST_ASSERT_EQUAL_UINT8(255, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(0, d_train.get_right_duty_cycle());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move forward.
    ============================================================
*/
void test_drive_train_forward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move backward.
    ============================================================
*/
void test_drive_train_backward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn left.
    ============================================================
*/
void test_drive_train_left_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn right.
    ============================================================
*/
void test_drive_train_right_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train turned right.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_right_duty_cycle());

    d_train.right_turn();

    TEST_ASSERT_EQUAL(Train_Command::RIGHT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can stop.
    ============================================================
*/
void test_drive_train_stop(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());

    // Verify the drive train stopped moving.
    d_train.set_left_duty_cycle(100);
    d_train.set_right_duty_cycle(100);

    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(100, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move forward and then forward.
    ============================================================
*/
void test_drive_train_forward_to_forward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move forward and then
    backward.
    ============================================================
*/
void test_drive_train_forward_to_backward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move forward and then turn
    left.
    ============================================================
*/
void test_drive_train_forward_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move forward and then turn
    right.
    ============================================================
*/
void test_drive_train_forward_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train turned right.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.right_turn();

    TEST_ASSERT_EQUAL(Train_Command::RIGHT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move forward and then stop.
    ============================================================
*/
void test_drive_train_forward_to_stop(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move backward and then
    forward.
    ============================================================
*/
void test_drive_train_backward_to_forward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move backward and then
    backward.
    ============================================================
*/
void test_drive_train_backward_to_backward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move backward and then turn
    left.
    ============================================================
*/
void test_drive_train_backward_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move backward and then turn
    right.
    ============================================================
*/
void test_drive_train_backward_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train turned right.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.right_turn();

    TEST_ASSERT_EQUAL(Train_Command::RIGHT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can move backward and then stop.
    ============================================================
*/
void test_drive_train_backward_to_stop(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn left and then forward.
    ============================================================
*/
void test_drive_train_left_turn_to_forward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn left and then backward.
    ============================================================
*/
void test_drive_train_left_turn_to_backward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn left and then turn left.
    ============================================================
*/
void test_drive_train_left_turn_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn left and then turn right.
    ============================================================
*/
void test_drive_train_left_turn_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train turned right.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.right_turn();

    TEST_ASSERT_EQUAL(Train_Command::RIGHT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can turn left and then stop.
    ============================================================
*/
void test_drive_train_left_turn_to_stop(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can stop and then forward.
    ============================================================
*/
void test_drive_train_stop_to_forward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());


    // Verify the drive train moved forward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.forward();

    TEST_ASSERT_EQUAL(Train_Command::FORWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::FORWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can stop and then backward.
    ============================================================
*/
void test_drive_train_stop_to_backward(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());


    // Verify the drive train moved backward.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.backward();

    TEST_ASSERT_EQUAL(Train_Command::BACKWARD, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::BACKWARD, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can stop and then turn left.
    ============================================================
*/
void test_drive_train_stop_to_left_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.left_turn();

    TEST_ASSERT_EQUAL(Train_Command::LEFT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::LEFT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can stop and then turn right.
    ============================================================
*/
void test_drive_train_stop_to_right_turn(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train turned left.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());


    // Verify the drive train turned right.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.right_turn();

    TEST_ASSERT_EQUAL(Train_Command::RIGHT_TURN, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::RIGHT_TURN, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::FORWARD, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::BACKWARD, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the drive train can stop and then stop.
    ============================================================
*/
void test_drive_train_stop_to_stop(void) {
    // Create the motor objects.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Verify that all motors are inititalized.
    TEST_ASSERT_TRUE(fl_motor.is_initialized());
    TEST_ASSERT_TRUE(fr_motor.is_initialized());
    TEST_ASSERT_TRUE(rl_motor.is_initialized());
    TEST_ASSERT_TRUE(rr_motor.is_initialized());
    TEST_ASSERT_FALSE(fl_motor.is_faulted());
    TEST_ASSERT_FALSE(fr_motor.is_faulted());
    TEST_ASSERT_FALSE(rl_motor.is_faulted());
    TEST_ASSERT_FALSE(rr_motor.is_faulted());

    // Create the motor driver objects.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Verify that all motor drivers are initialized.
    TEST_ASSERT_TRUE(fm_driver.is_initialized());
    TEST_ASSERT_TRUE(rm_driver.is_initialized());
    TEST_ASSERT_FALSE(fm_driver.is_faulted());
    TEST_ASSERT_FALSE(rm_driver.is_faulted());

    // Create the wheel encoder objects.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Verify that all wheel encoders are initialized.
    TEST_ASSERT_TRUE(lw_encoder.is_initialized());
    TEST_ASSERT_TRUE(rw_encoder.is_initialized());
    TEST_ASSERT_FALSE(lw_encoder.is_faulted());
    TEST_ASSERT_FALSE(rw_encoder.is_faulted());

    // Create the drive train object.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};
    Drive_Train d_train(dt_config);

    // Verify that the drive train is initialized.
    TEST_ASSERT_TRUE(d_train.is_initialized());
    TEST_ASSERT_FALSE(d_train.is_faulted());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(123);
    d_train.set_right_duty_cycle(123);

    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(123, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());


    // Verify the drive train stopped.
    d_train.set_left_duty_cycle(231);
    d_train.set_right_duty_cycle(231);

    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_left_duty_cycle());
    TEST_ASSERT_EQUAL_UINT8(231, d_train.get_right_duty_cycle());

    d_train.stop();

    TEST_ASSERT_EQUAL(Train_Command::STOP, d_train.get_train_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_front_driver_command());
    TEST_ASSERT_EQUAL(Motor_Driver_Command::STOP, d_train.get_rear_driver_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_fd_right_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_left_motor_command());
    TEST_ASSERT_EQUAL(Motor_Command::STOP, d_train.get_rd_right_motor_command());

    return;
}
//  ============================================================