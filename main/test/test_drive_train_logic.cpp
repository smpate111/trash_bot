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

#include <motors/motor.hpp>
#include <motors/motor_driver.hpp>
#include <controllers/drive_train.hpp>

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



/*
    ============================================================
    Test the drive train class and its functions.
    ============================================================
*/
void test_drive_train_functions(void) {
    // Create the motor objects.
    Motor_Config fl_config = {
        "Front Left Motor",
        GPIO_NUM_11, GPIO_NUM_12,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1
    };

    Motor_Config fr_config = {
        "Front Right Motor",
        GPIO_NUM_13, GPIO_NUM_14,
        LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };

    Motor_Config bl_config = {
        "Back Left Motor",
        GPIO_NUM_6, GPIO_NUM_7,
        LEDC_CHANNEL_5, LEDC_CHANNEL_4
    };

    Motor_Config br_config = {
        "Back Right Motor",
        GPIO_NUM_15, GPIO_NUM_16,
        LEDC_CHANNEL_7, LEDC_CHANNEL_6
    };

    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor bl_motor(bl_config);
    Motor br_motor(br_config);

    // Create the motor driver objects.
    Driver_Config fd_config = {
        "Front Driver",
        fl_motor, fr_motor
    };

    Driver_Config bd_config = {
        "Back Driver",
        bl_motor, br_motor
    };

    Motor_Driver f_driver(fd_config);
    Motor_Driver b_driver(bd_config);

    // Create the wheel encoder objects.
    Encoder_Config le_config = {
        "Left Encoder", GPIO_NUM_1,
        80.0, 20
    };

    Encoder_Config re_config = {
        "Right Encoder", GPIO_NUM_2,
        80.0, 20
    };

    Wheel_Encoder l_encoder(le_config);
    Wheel_Encoder r_encoder(re_config);

    // Configure the drive train object.
    Train_Config t_config = {
        "Drive Train",
        f_driver, b_driver,
        &l_encoder, &r_encoder
    };

    Drive_Train d_train(t_config);


    /*
        ============================================================
        Test 1: Verify the hardware initialization of the drive
        train.
        ============================================================
    */

    // Confirm the GPIO pins were reset.
    TEST_ASSERT_EQUAL(10, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_reset_pin_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_reset_pin_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_NUM_13, gpio_reset_pin_fake.arg0_history[2]);
    TEST_ASSERT_EQUAL(GPIO_NUM_14, gpio_reset_pin_fake.arg0_history[3]);
    TEST_ASSERT_EQUAL(GPIO_NUM_6, gpio_reset_pin_fake.arg0_history[4]);
    TEST_ASSERT_EQUAL(GPIO_NUM_7, gpio_reset_pin_fake.arg0_history[5]);
    TEST_ASSERT_EQUAL(GPIO_NUM_15, gpio_reset_pin_fake.arg0_history[6]);
    TEST_ASSERT_EQUAL(GPIO_NUM_16, gpio_reset_pin_fake.arg0_history[7]);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_reset_pin_fake.arg0_history[8]);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_reset_pin_fake.arg0_history[9]);

    // Confirm the GPIO pins' directions.
    TEST_ASSERT_EQUAL(10, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_11, gpio_set_direction_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_12, gpio_set_direction_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[1]);
    TEST_ASSERT_EQUAL(GPIO_NUM_13, gpio_set_direction_fake.arg0_history[2]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[2]);
    TEST_ASSERT_EQUAL(GPIO_NUM_14, gpio_set_direction_fake.arg0_history[3]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[3]);
    TEST_ASSERT_EQUAL(GPIO_NUM_6, gpio_set_direction_fake.arg0_history[4]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[4]);
    TEST_ASSERT_EQUAL(GPIO_NUM_7, gpio_set_direction_fake.arg0_history[5]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[5]);
    TEST_ASSERT_EQUAL(GPIO_NUM_15, gpio_set_direction_fake.arg0_history[6]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[6]);
    TEST_ASSERT_EQUAL(GPIO_NUM_16, gpio_set_direction_fake.arg0_history[7]);
    TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[7]);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_set_direction_fake.arg0_history[8]);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[8]);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_set_direction_fake.arg0_history[9]);
    TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[9]);

    // Confirm the number of calls for the LEDC config.
    TEST_ASSERT_EQUAL(8, ledc_channel_config_fake.call_count);

    // Confirm the number of calls for the GPIO config.
    TEST_ASSERT_EQUAL(2, gpio_config_fake.call_count);

    // Confirm the number of calls for the ISR handler and which GPIO pin called it.
    TEST_ASSERT_EQUAL(2, gpio_isr_handler_add_fake.call_count);
    TEST_ASSERT_EQUAL(GPIO_NUM_1, gpio_isr_handler_add_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(GPIO_NUM_2, gpio_isr_handler_add_fake.arg0_history[1]);
    //  ============================================================


    /*
        ============================================================
        Test 2: Check if the fake LEDC hardware is correctly called
        in move_forward(). Also verify the Interrupt Service Routine
        gets triggered and performs pulse counting.
        ============================================================
    */

    d_train.change_speed(255, 255);
    d_train.move_forward();

    // Assume that we are moving forward for 3 seconds.
    uint32_t duration = 3;

    // The number of pulses counted per second based on the speed of the motors.
    // This number is based on the actual hardware and will need to be adjusted if the
    // speed or hardware changes.
    uint32_t pulses_per_second = 30;

    // Verify the pulse count is 0 before performing the count.
    l_encoder.reset_count();
    r_encoder.reset_count();
    TEST_ASSERT_EQUAL(0, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(0, r_encoder.pulse_count);

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(8, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the front left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[0]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[0]);       // config.Front_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[0]);     // Current_Speed

    // Confirm the LEDC duty for the front left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[1]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[1]);       // config.Front_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[1]);     // 0.0

    // Confirm the LEDC duty for the front right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[2]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[2]);       // config.Front_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[2]);     // Current_Speed

    // Confirm the LEDC duty for the front right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[3]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[3]);       // config.Front_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[3]);     // 0.0

    // Confirm the LEDC duty for the back left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[4]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(5, ledc_set_duty_fake.arg1_history[4]);       // config.Back_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[4]);     // Current_Speed

    // Confirm the LEDC duty for the back left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[5]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.arg1_history[5]);       // config.Back_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[5]);     // 0.0

    // Confirm the LEDC duty for the back right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[6]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(7, ledc_set_duty_fake.arg1_history[6]);       // config.Back_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(255, ledc_set_duty_fake.arg2_history[6]);     // Current_Speed

    // Confirm the LEDC duty for the back right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[7]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(6, ledc_set_duty_fake.arg1_history[7]);       // config.Back_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[7]);     // 0.0

    // Multiply the duration by the number of pulses per second to simulate the ISR handler being called
    // that many times per second.
    for (uint32_t i = 0; i < (duration * pulses_per_second); i++) {
        Wheel_Encoder::isr_handler(&l_encoder);
        Wheel_Encoder::isr_handler(&r_encoder);
    }

    // Confirm the pulse count after moving forward for the specified duration.
    TEST_ASSERT_EQUAL(90, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(90, r_encoder.pulse_count);
    //  ============================================================


    /*
        ============================================================
        Test 2: Check if the fake LEDC hardware is correctly called
        in move_backward(). Also verify the Interrupt Service Routine
        gets triggered and performs pulse counting.
        ============================================================
    */

    d_train.change_speed(200, 200);
    d_train.move_backward();

    // Assume that we are moving backward for 3 seconds.
    duration = 3;

    // The number of pulses counted per second based on the speed of the motors.
    // This number is based on the actual hardware and will need to be adjusted if the
    // speed or hardware changes.
    pulses_per_second = 25;

    // Verify the pulse count is 0 before performing the count.
    l_encoder.reset_count();
    r_encoder.reset_count();
    TEST_ASSERT_EQUAL(0, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(0, r_encoder.pulse_count);

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(16, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the front left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[8]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[8]);       // config.Front_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[8]);     // 0.0

    // Confirm the LEDC duty for the front left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[9]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[9]);       // config.Front_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[9]);     // Current_Speed

    // Confirm the LEDC duty for the front right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[10]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[10]);       // config.Front_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[10]);     // 0.0

    // Confirm the LEDC duty for the front right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[11]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[11]);       // config.Front_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[11]);     // Current_Speed

    // Confirm the LEDC duty for the back left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[12]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(5, ledc_set_duty_fake.arg1_history[12]);       // config.Back_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[12]);     // 0.0

    // Confirm the LEDC duty for the back left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[13]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.arg1_history[13]);       // config.Back_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[13]);     // Current_Speed

    // Confirm the LEDC duty for the back right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[14]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(7, ledc_set_duty_fake.arg1_history[14]);       // config.Back_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[14]);     // 0.0

    // Confirm the LEDC duty for the back right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[15]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(6, ledc_set_duty_fake.arg1_history[15]);       // config.Back_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(200, ledc_set_duty_fake.arg2_history[15]);     // Current_Speed

    // Multiply the duration by the number of pulses per second to simulate the ISR handler being called
    // that many times per second.
    for (uint32_t i = 0; i < (duration * pulses_per_second); i++) {
        Wheel_Encoder::isr_handler(&l_encoder);
        Wheel_Encoder::isr_handler(&r_encoder);
    }

    // Confirm the pulse count after moving forward for the specified duration.
    TEST_ASSERT_EQUAL(75, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(75, r_encoder.pulse_count);
    //  ============================================================


    /*
        ============================================================
        Test 3: Check if the fake LEDC hardware is correctly called
        in turn_left(). Also verify the Interrupt Service Routine
        gets triggered and performs pulse counting.
        ============================================================
    */

    d_train.change_speed(225, 225);
    d_train.turn_left();

    // Assume that we are turning left for 3 seconds.
    duration = 3;

    // The number of pulses counted per second based on the speed of the motors.
    // This number is based on the actual hardware and will need to be adjusted if the
    // speed or hardware changes.
    pulses_per_second = 28;

    // Verify the pulse count is 0 before performing the count.
    l_encoder.reset_count();
    r_encoder.reset_count();
    TEST_ASSERT_EQUAL(0, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(0, r_encoder.pulse_count);

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(24, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the front left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[16]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[16]);       // config.Front_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[16]);     // 0.0

    // Confirm the LEDC duty for the front left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[17]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[17]);       // config.Front_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(225, ledc_set_duty_fake.arg2_history[17]);     // Current_Speed

    // Confirm the LEDC duty for the front right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[18]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[18]);       // config.Front_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(225, ledc_set_duty_fake.arg2_history[18]);     // Current_Speed

    // Confirm the LEDC duty for the front right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[19]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[19]);       // config.Front_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[19]);     // 0.0

    // Confirm the LEDC duty for the back left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[20]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(5, ledc_set_duty_fake.arg1_history[20]);       // config.Back_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[20]);     // 0.0

    // Confirm the LEDC duty for the back left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[21]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.arg1_history[21]);       // config.Back_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(225, ledc_set_duty_fake.arg2_history[21]);     // Current_Speed

    // Confirm the LEDC duty for the back right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[22]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(7, ledc_set_duty_fake.arg1_history[22]);       // config.Back_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(225, ledc_set_duty_fake.arg2_history[22]);     // Current_Speed

    // Confirm the LEDC duty for the back right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[23]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(6, ledc_set_duty_fake.arg1_history[23]);       // config.Back_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[23]);     // 0.0

    // Multiply the duration by the number of pulses per second to simulate the ISR handler being called
    // that many times per second.
    for (uint32_t i = 0; i < (duration * pulses_per_second); i++) {
        Wheel_Encoder::isr_handler(&l_encoder);
        Wheel_Encoder::isr_handler(&r_encoder);
    }

    // Confirm the pulse count after moving forward for the specified duration.
    TEST_ASSERT_EQUAL(84, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(84, r_encoder.pulse_count);
    //  ============================================================


    /*
        ============================================================
        Test 4: Check if the fake LEDC hardware is correctly called
        in turn_right(). Also verify the Interrupt Service Routine
        gets triggered and performs pulse counting.
        ============================================================
    */

    d_train.change_speed(210, 210);
    d_train.turn_right();

    // Assume that we are turning right for 3 seconds.
    duration = 3;

    // The number of pulses counted per second based on the speed of the motors.
    // This number is based on the actual hardware and will need to be adjusted if the
    // speed or hardware changes.
    pulses_per_second = 27;

    // Verify the pulse count is 0 before performing the count.
    l_encoder.reset_count();
    r_encoder.reset_count();
    TEST_ASSERT_EQUAL(0, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(0, r_encoder.pulse_count);

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(32, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the front left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[24]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[24]);       // config.Front_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(210, ledc_set_duty_fake.arg2_history[24]);     // Current_Speed

    // Confirm the LEDC duty for the front left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[25]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[25]);       // config.Front_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[25]);     // 0.0

    // Confirm the LEDC duty for the front right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[26]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[26]);       // config.Front_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[26]);     // 0.0

    // Confirm the LEDC duty for the front right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[27]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[27]);       // config.Front_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(210, ledc_set_duty_fake.arg2_history[27]);     // Current_Speed

    // Confirm the LEDC duty for the back left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[28]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(5, ledc_set_duty_fake.arg1_history[28]);       // config.Back_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(210, ledc_set_duty_fake.arg2_history[28]);     // Current_Speed

    // Confirm the LEDC duty for the back left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[29]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.arg1_history[29]);       // config.Back_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[29]);     // 0.0

    // Confirm the LEDC duty for the back right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[30]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(7, ledc_set_duty_fake.arg1_history[30]);       // config.Back_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[30]);     // 0.0

    // Confirm the LEDC duty for the back right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[31]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(6, ledc_set_duty_fake.arg1_history[31]);       // config.Back_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(210, ledc_set_duty_fake.arg2_history[31]);     // Current_Speed

    // Multiply the duration by the number of pulses per second to simulate the ISR handler being called
    // that many times per second.
    for (uint32_t i = 0; i < (duration * pulses_per_second); i++) {
        Wheel_Encoder::isr_handler(&l_encoder);
        Wheel_Encoder::isr_handler(&r_encoder);
    }

    // Confirm the pulse count after moving forward for the specified duration.
    TEST_ASSERT_EQUAL(81, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(81, r_encoder.pulse_count);
    //  ============================================================

    
    /*
        ============================================================
        Test 5: Check if the fake LEDC hardware is correctly called
        in turn_right(). Also verify the Interrupt Service Routine
        gets triggered and performs pulse counting.
        ============================================================
    */

    d_train.brake_all();

    // Verify the pulse count is 0.
    l_encoder.reset_count();
    r_encoder.reset_count();
    TEST_ASSERT_EQUAL(0, l_encoder.pulse_count);
    TEST_ASSERT_EQUAL(0, r_encoder.pulse_count);

    // Confirm the number of calls for the LEDC duty.
    TEST_ASSERT_EQUAL(40, ledc_set_duty_fake.call_count);

    // Confirm the LEDC duty for the front left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[32]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg1_history[32]);       // config.Front_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[32]);     // 0.0

    // Confirm the LEDC duty for the front left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[33]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(1, ledc_set_duty_fake.arg1_history[33]);       // config.Front_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[33]);     // 0.0

    // Confirm the LEDC duty for the front right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[34]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(2, ledc_set_duty_fake.arg1_history[34]);       // config.Front_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[34]);     // 0.0

    // Confirm the LEDC duty for the front right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[35]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(3, ledc_set_duty_fake.arg1_history[35]);       // config.Front_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[35]);     // 0.0

    // Confirm the LEDC duty for the back left motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[36]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(5, ledc_set_duty_fake.arg1_history[36]);       // config.Back_Left_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[36]);     // 0.0

    // Confirm the LEDC duty for the back left motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[37]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(4, ledc_set_duty_fake.arg1_history[37]);       // config.Back_Left_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[37]);     // 0.0

    // Confirm the LEDC duty for the back right motor's channel 1.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[38]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(7, ledc_set_duty_fake.arg1_history[38]);       // config.Back_Right_Motor.channel_1
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[38]);     // 0.0

    // Confirm the LEDC duty for the back right motor's channel 2.
    TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[39]);       // LEDC_LOW_SPEED_MODE
    TEST_ASSERT_EQUAL(6, ledc_set_duty_fake.arg1_history[39]);       // config.Back_Right_Motor.channel_2
    TEST_ASSERT_EQUAL(0.0, ledc_set_duty_fake.arg2_history[39]);     // 0.0
    //  ============================================================

    return;
}