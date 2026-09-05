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



// Global variables that tracks hardware configurations.
constexpr int number_of_pins = 10;
constexpr int number_of_ledc_channels = 8;
constexpr int number_of_gpio_configs = 2;
constexpr int number_of_interrupts = 2;
static int pins[number_of_pins];
static int ledc_channels[number_of_ledc_channels];
static double expected_duties[number_of_ledc_channels];
static int interrupts[number_of_interrupts];

enum movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};



/*
    ============================================================
    Helper function that creates the drive train object.
    ============================================================
*/
Drive_Train create_drive_train() {
    // Create the motor objects.
    static Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    static Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_13, GPIO_NUM_14, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    static Motor_Config bl_config = {"Back Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    static Motor_Config br_config = {"Back Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};

    static Motor fl_motor(fl_config);
    static Motor fr_motor(fr_config);
    static Motor bl_motor(bl_config);
    static Motor br_motor(br_config);

    // Create the motor driver objects.
    static Driver_Config fd_config = {"Front Driver", fl_motor, fr_motor};
    static Driver_Config bd_config = {"Back Driver", bl_motor, br_motor};

    static Motor_Driver f_driver(fd_config);
    static Motor_Driver b_driver(bd_config);

    // Create the wheel encoder objects.
    static Encoder_Config le_config = {"Left Encoder", GPIO_NUM_1, 80.0, 20};
    static Encoder_Config re_config = {"Right Encoder", GPIO_NUM_2, 80.0, 20};

    static Wheel_Encoder l_encoder(le_config);
    static Wheel_Encoder r_encoder(re_config);

    // Configure the drive train object.
    static Train_Config t_config = {"Drive Train", f_driver, b_driver, &l_encoder, &r_encoder};

    pins[0] = GPIO_NUM_11;
    pins[1] = GPIO_NUM_12;
    pins[2] = GPIO_NUM_13;
    pins[3] = GPIO_NUM_14;
    pins[4] = GPIO_NUM_6;
    pins[5] = GPIO_NUM_7;
    pins[6] = GPIO_NUM_15;
    pins[7] = GPIO_NUM_16;
    pins[8] = GPIO_NUM_1;
    pins[9] = GPIO_NUM_2;

    ledc_channels[0] = LEDC_CHANNEL_0;
    ledc_channels[1] = LEDC_CHANNEL_1;
    ledc_channels[2] = LEDC_CHANNEL_2;
    ledc_channels[3] = LEDC_CHANNEL_3;
    ledc_channels[4] = LEDC_CHANNEL_5;
    ledc_channels[5] = LEDC_CHANNEL_4;
    ledc_channels[6] = LEDC_CHANNEL_7;
    ledc_channels[7] = LEDC_CHANNEL_6;

    interrupts[0] = GPIO_NUM_1;
    interrupts[1] = GPIO_NUM_2;

    return Drive_Train(t_config);
}
//  ============================================================



/*
    ============================================================
    Confirm that the drive train's constructor initializes
    the object correctly. Confirm that start_task() creates a
    FreeRTOS task.
    ============================================================
*/
void test_drive_train_initialization(Drive_Train &d_train) {
    // Verify the drive train hardware is initialized.
    for (int i = 0; i < number_of_pins; i++) {
        TEST_ASSERT_EQUAL(pins[i], gpio_reset_pin_fake.arg0_history[i]);
        TEST_ASSERT_EQUAL(pins[i], gpio_set_direction_fake.arg0_history[i]);

        if (i < 8) {
            TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, gpio_set_direction_fake.arg1_history[i]);
        }
        else if ((i >= 8) && (i < 10)) {
            TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, gpio_set_direction_fake.arg1_history[i]);
        }
    }

    for (int i = 0; i < number_of_interrupts; i++) {
        TEST_ASSERT_EQUAL(interrupts[i], gpio_isr_handler_add_fake.arg0_history[i]);
    }

    TEST_ASSERT_EQUAL(number_of_pins, gpio_reset_pin_fake.call_count);
    TEST_ASSERT_EQUAL(number_of_pins, gpio_set_direction_fake.call_count);
    TEST_ASSERT_EQUAL(number_of_ledc_channels, ledc_channel_config_fake.call_count);
    TEST_ASSERT_EQUAL(number_of_interrupts, gpio_isr_handler_add_fake.call_count);
    TEST_ASSERT_EQUAL(number_of_gpio_configs, gpio_config_fake.call_count);

    // Verify the Odometry Task is created.
    d_train.start_task();
    TEST_ASSERT_EQUAL(1, xTaskCreate_fake.call_count);
    TEST_ASSERT_EQUAL_STRING("Odometry_Task", xTaskCreate_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(4096, xTaskCreate_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(&d_train, xTaskCreate_fake.arg3_history[0]);
    TEST_ASSERT_EQUAL(4, xTaskCreate_fake.arg4_history[0]);
    TEST_ASSERT_EQUAL(nullptr, xTaskCreate_fake.arg5_history[0]);

    return;
}
//  ============================================================



/*
    ============================================================
    Confirm that the drive train performs the correct movements.
    ============================================================
*/
void test_drive_train_movement(
        Drive_Train &d_train, movement command, int speed, uint32_t duration,
        uint32_t pulses_per_second, int ledc_calls
    ) {
        // Verify the pulse count is 0 before performing the count.
        Wheel_Encoder l_encoder = d_train.get_left_encoder();
        Wheel_Encoder r_encoder = d_train.get_right_encoder();

        l_encoder.reset_count();
        r_encoder.reset_count();
        
        TEST_ASSERT_EQUAL(0, d_train.get_pulses(l_encoder));
        TEST_ASSERT_EQUAL(0, d_train.get_pulses(r_encoder));

        // Verify the movement.
        d_train.change_speed(speed, speed);

        switch(command) {
            case movement::FORWARD:
                d_train.move_forward();
                break;

            case movement::BACKWARD:
                d_train.move_backward();
                break;

            case movement::LEFT:
                d_train.turn_left();
                break;

            case movement::RIGHT:
                d_train.turn_right();
                break;

            case movement::STOP:
                d_train.brake_all();
                break;
        }

        TEST_ASSERT_EQUAL(ledc_calls, ledc_set_duty_fake.call_count);

        for (int i = 0; i < number_of_ledc_channels; i++) {
            int history_i = i;
            
            if (ledc_calls > 9) {
                history_i = history_i + (ledc_calls - 8);
            }
            
            TEST_ASSERT_EQUAL(0, ledc_set_duty_fake.arg0_history[history_i]);
            TEST_ASSERT_EQUAL(ledc_channels[i], ledc_set_duty_fake.arg1_history[history_i]);
            TEST_ASSERT_EQUAL(expected_duties[i], ledc_set_duty_fake.arg2_history[history_i]);
        }

        // Simulate the ISR when the robot is moving.
        if (command != movement::STOP) {
            // Multiply the duration by the number of pulses per second to estimate the ISR was called that many times.
            for (uint32_t i = 0; i < (duration * pulses_per_second); i++) {
                Wheel_Encoder::isr_handler(&l_encoder);
                Wheel_Encoder::isr_handler(&r_encoder);
            }

            // Confirm the pulse count after moving for the specified duration.
            TEST_ASSERT_EQUAL(duration * pulses_per_second, d_train.get_pulses(l_encoder));
            TEST_ASSERT_EQUAL(duration * pulses_per_second, d_train.get_pulses(r_encoder));
        }
        return;
}
//  ============================================================



/*
    ============================================================
    Confirm that the drive train performs the loop_tick().
    ============================================================
*/
void test_drive_train_task(Drive_Train &d_train) {
    Wheel_Encoder &l_encoder = d_train.get_left_encoder();
    Wheel_Encoder &r_encoder = d_train.get_right_encoder();
    
    // Reset the pulse counts.
    l_encoder.reset_count();
    r_encoder.reset_count();
    TEST_ASSERT_EQUAL(0, d_train.get_pulses(l_encoder));
    TEST_ASSERT_EQUAL(0, d_train.get_pulses(r_encoder));

    // Simulate the wheel encoders pulsed 20 times.
    l_encoder.set_pulse_count(20);
    r_encoder.set_pulse_count(20);
    
    // Confirm the robot is moving.
    d_train.loop_tick();
    TEST_ASSERT_EQUAL(d_train.get_last_l_pulses(), d_train.get_pulses(l_encoder));
    TEST_ASSERT_EQUAL(d_train.get_last_r_pulses(), d_train.get_pulses(r_encoder));
    TEST_ASSERT_EQUAL(false, d_train.stopped_recently);

    // Confirm the robot stopped moving.
    d_train.loop_tick();
    TEST_ASSERT_EQUAL(0, d_train.get_pulses(l_encoder));
    TEST_ASSERT_EQUAL(0, d_train.get_pulses(r_encoder));
    TEST_ASSERT_EQUAL(0, d_train.get_last_l_pulses());
    TEST_ASSERT_EQUAL(0, d_train.get_last_r_pulses());
    TEST_ASSERT_EQUAL(0.0, d_train.get_l_distance());
    TEST_ASSERT_EQUAL(0.0, d_train.get_r_distance());
    TEST_ASSERT_EQUAL(true, d_train.stopped_recently);

    return;
}
//  ============================================================



/*
    ============================================================
    Test the drive train class and its functions.
    ============================================================
*/
void test_drive_train_functions(void) {
    // Create the drive train object.
    Drive_Train d_train = create_drive_train();

    // Confirm that the drive train is initialized and its task is created.
    test_drive_train_initialization(d_train);

    // Confirm the robot moved forward.
    // expected motor duties in order: fl_1, fl_2, fr_1, fr_2, bl_1, bl_2, br_1, br_2
    expected_duties[0] = 255;
    expected_duties[1] = 0.0;
    expected_duties[2] = 255;
    expected_duties[3] = 0.0;
    expected_duties[4] = 255;
    expected_duties[5] = 0.0;
    expected_duties[6] = 255;
    expected_duties[7] = 0.0;
    test_drive_train_movement(d_train, movement::FORWARD, 255, 3, 30, 8);

    // Confirm the robot moved backward.
    // expected motor duties in order: fl_1, fl_2, fr_1, fr_2, bl_1, bl_2, br_1, br_2
    expected_duties[0] = 0.0;
    expected_duties[1] = 200;
    expected_duties[2] = 0.0;
    expected_duties[3] = 200;
    expected_duties[4] = 0.0;
    expected_duties[5] = 200;
    expected_duties[6] = 0.0;
    expected_duties[7] = 200;
    test_drive_train_movement(d_train, movement::BACKWARD, 200, 3, 25, 16);

    // Confirm the robot turned left.
    // expected motor duties in order: fl_1, fl_2, fr_1, fr_2, bl_1, bl_2, br_1, br_2
    expected_duties[0] = 0.0;
    expected_duties[1] = 225;
    expected_duties[2] = 225;
    expected_duties[3] = 0.0;
    expected_duties[4] = 0.0;
    expected_duties[5] = 225;
    expected_duties[6] = 225;
    expected_duties[7] = 0.0;
    test_drive_train_movement(d_train, movement::LEFT, 225, 3, 28, 24);

    // Confirm the robot turned right.
    // expected motor duties in order: fl_1, fl_2, fr_1, fr_2, bl_1, bl_2, br_1, br_2
    expected_duties[0] = 210;
    expected_duties[1] = 0.0;
    expected_duties[2] = 0.0;
    expected_duties[3] = 210;
    expected_duties[4] = 210;
    expected_duties[5] = 0.0;
    expected_duties[6] = 0.0;
    expected_duties[7] = 210;
    test_drive_train_movement(d_train, movement::RIGHT, 210, 3, 27, 32);

    // Confirm the robot stopped.
    // expected motor duties in order: fl_1, fl_2, fr_1, fr_2, bl_1, bl_2, br_1, br_2
    expected_duties[0] = 0.0;
    expected_duties[1] = 0.0;
    expected_duties[2] = 0.0;
    expected_duties[3] = 0.0;
    expected_duties[4] = 0.0;
    expected_duties[5] = 0.0;
    expected_duties[6] = 0.0;
    expected_duties[7] = 0.0;
    test_drive_train_movement(d_train, movement::STOP, 0, 3, 100, 40);

    // Confirm the drive train task works.
    test_drive_train_task(d_train);

    return;
}