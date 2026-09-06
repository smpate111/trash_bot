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

#include <controllers/drive_train.hpp>
#include <controllers/robot_controller.hpp>
#include <controllers/obstacle_avoidance.hpp>

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



// Define a global variable that stores a copy of the command sent to the FreeRTOS queue during testing.
extern Robot_Command sent_command;
extern Robot_Command sent_emergency_command;
extern bool queue_received_command;



/*
    ============================================================
    Helper function that creates the drive train object.
    ============================================================
*/
Drive_Train create_drive_train_2() {
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

    return Drive_Train(t_config);
}
//  ============================================================



/*
    ============================================================
    A fake function used to capture the command sent to the
    FreeRTOS queue during testing before it goes out of scope.
    ============================================================
*/
extern BaseType_t custom_xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks);
//  ============================================================



/*
    ============================================================
    A fake function used to capture the command sent to the
    front of the FreeRTOS queue during testing before it goes
    out of scope.
    ============================================================
*/
extern BaseType_t custom_xQueueSendToFront(QueueHandle_t queue, const void *item, TickType_t ticks);
//  ============================================================



/*
    ============================================================
    A fake function used by the FreeRTOS queue that receives the
    command.
    ============================================================
*/
extern BaseType_t custom_xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks);
//  ============================================================



/*
    ============================================================
    Confirm that the obstacle avoidance's constructor
    initializes the object correctly. Confirm that start_task()
    creates a FreeRTOS task.
    ============================================================
*/
void test_obstacle_avoidance_initialization(Obstacle_Avoidance &o_avoidance) {
    o_avoidance.start_task();
    TEST_ASSERT_EQUAL(1, xTaskCreate_fake.call_count);
    TEST_ASSERT_EQUAL_STRING("Obstacle_Avoidance_Task", xTaskCreate_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(4096, xTaskCreate_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(&o_avoidance, xTaskCreate_fake.arg3_history[0]);
    TEST_ASSERT_EQUAL(5, xTaskCreate_fake.arg4_history[0]);
    TEST_ASSERT_EQUAL(nullptr, xTaskCreate_fake.arg5_history[0]);

    return;
}
//  ============================================================



/*
    ============================================================
    Confirm that standard movements are not interrupted when
    the active sensor detects a clear path.
    ============================================================
*/
void test_clear_path(Obstacle_Avoidance &o_avoidance, Robot_Controller &r_controller, Ultrasonic &us) {
    int initial_reset_calls = xQueueReset_fake.call_count;
    us.is_testing = true;

    us.set_distance(500.0);
    TEST_ASSERT_EQUAL_DOUBLE(500.0, us.get_distance());

    r_controller.set_robot_state(Robot_State::MOVE_FORWARD);
    o_avoidance.loop_tick();

    TEST_ASSERT_EQUAL(initial_reset_calls, xQueueReset_fake.call_count);
    return;
}
//  ============================================================



/*
    ============================================================
    Confirm that the robot only listens to a specific sensor
    that corresponds to its current movement. If a different
    sensor detects an object past the distance threshold, the
    robot won't trigger an emergency stop.
    ============================================================
*/
void test_state_sensor_mapping(Obstacle_Avoidance &o_avoidance, Robot_Controller &r_controller, Ultrasonic &us) {
    int initial_reset_calls = xQueueReset_fake.call_count;
    us.is_testing = true;

    us.set_distance(20.0);
    TEST_ASSERT_EQUAL_DOUBLE(20.0, us.get_distance());

    r_controller.set_robot_state(Robot_State::MOVE_FORWARD);
    o_avoidance.loop_tick();

    TEST_ASSERT_EQUAL(initial_reset_calls, xQueueReset_fake.call_count);
    return;
}
//  ============================================================



/*
    ============================================================
    Confirm that the robot does not trigger an emergency stop
    when it is in the brake state because the ultrasonic sensors
    aren't turned on when in this state.
    ============================================================
*/
void test_brake_state(Obstacle_Avoidance &o_avoidance, Robot_Controller &r_controller, Ultrasonic &us) {
    int initial_reset_calls = xQueueReset_fake.call_count;
    us.is_testing = true;

    us.set_distance(20.0);
    TEST_ASSERT_EQUAL_DOUBLE(20.0, us.get_distance());

    r_controller.set_robot_state(Robot_State::BRAKE);
    o_avoidance.loop_tick();

    TEST_ASSERT_EQUAL(initial_reset_calls, xQueueReset_fake.call_count);
    return;
}
//  ============================================================



/*
    ============================================================
    Confirm that the robot triggers an emergency stop during a
    movement with the corresponding active ultrasonic sensor.
    ============================================================
*/
void test_emergency_stop(Obstacle_Avoidance &o_avoidance, Robot_Controller &r_controller, Ultrasonic &us) {
    int initial_reset_calls = xQueueReset_fake.call_count;
    us.is_testing = true;

    us.set_distance(20.0);
    TEST_ASSERT_EQUAL_DOUBLE(20.0, us.get_distance());

    r_controller.set_robot_state(Robot_State::MOVE_FORWARD);
    o_avoidance.loop_tick();

    TEST_ASSERT_EQUAL(initial_reset_calls + 1, xQueueReset_fake.call_count);
    return;
}
//  ============================================================



/*
    ============================================================
    Test the obstacle avoidance class and its functions.
    ============================================================
*/
void test_obstacle_avoidance_functions(void) {
    // Create the drive train and robot controller objects.
    Drive_Train d_train = create_drive_train_2();
    Robot_Controller r_controller(d_train);
    
    // Create the ultrasonic sensor objects.
    Ultrasonic_Config fus_config = {"Front Ultrasonic Sensor", GPIO_NUM_2, GPIO_NUM_42};
    Ultrasonic_Config bus_config = {"Back Ultrasonic Sensor", GPIO_NUM_5, GPIO_NUM_4};
    Ultrasonic_Config lus_config = {"Left Ultrasonic Sensor", GPIO_NUM_18, GPIO_NUM_17};
    Ultrasonic_Config rus_config = {"Right Ultrasonic Sensor", GPIO_NUM_9, GPIO_NUM_10};

    Ultrasonic f_us(fus_config);
    Ultrasonic b_us(bus_config);
    Ultrasonic l_us(lus_config);
    Ultrasonic r_us(rus_config);

    // Create the obstacle avoidance object.
    Avoid_Config a_config = {"Obstace Avoidance", &f_us, &b_us, &l_us, &r_us, &r_controller};
    Obstacle_Avoidance o_avoidance(a_config);

    xQueueSend_fake.custom_fake = custom_xQueueSend;
    xQueueSendToFront_fake.custom_fake = custom_xQueueSendToFront;
    xQueueReceive_fake.custom_fake = custom_xQueueReceive;

    // Confirm that the obstacle avoidance is initialized and its task is created.
    test_obstacle_avoidance_initialization(o_avoidance);
    
    // Confirm that the obstacle avoidance passes the following tests.
    test_clear_path(o_avoidance, r_controller, f_us);
    test_state_sensor_mapping(o_avoidance, r_controller, b_us);
    test_brake_state(o_avoidance, r_controller, f_us);
    test_emergency_stop(o_avoidance, r_controller, f_us);

    return;
}