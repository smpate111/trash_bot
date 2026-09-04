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

#include <test/mock_libraries/mock_hardware.hpp>

//  ============================================================



// Define a global variable that stores a copy of the command sent to the FreeRTOS queue during testing.
Robot_Command sent_command;
Robot_Command sent_emergency_command;
bool queue_received_command = false;



/*
    ============================================================
    Helper function that creates the drive train object.
    ============================================================
*/
Drive_Train create_drive_train_1() {
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
BaseType_t custom_xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks) {
    if (item != nullptr) {
        sent_command = *(const Robot_Command*)item;
        queue_received_command = true;
        return pdTRUE;
    }
    return pdFALSE;
}
//  ============================================================



/*
    ============================================================
    A fake function used to capture the command sent to the
    front of the FreeRTOS queue during testing before it goes
    out of scope.
    ============================================================
*/
BaseType_t custom_xQueueSendToFront(QueueHandle_t queue, const void *item, TickType_t ticks) {
    if (item != nullptr) {
        sent_emergency_command = *(const Robot_Command*)item;
        queue_received_command = true;
        return pdTRUE;
    }
    return pdFALSE;
}
//  ============================================================



/*
    ============================================================
    A fake function used by the FreeRTOS queue that receives the
    command.
    ============================================================
*/
BaseType_t custom_xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks) {
    if (queue_received_command == true) {
        if (item != nullptr) {
            *(Robot_Command*)item = sent_command;
        }
        queue_received_command = false;
        return pdTRUE;
    }
    return pdFALSE;
}
//  ============================================================



/*
    ============================================================
    Confirm that the robot controller's constructor initializes
    the object correctly, creates a command queue of size 10,
    and sets the robot's state to BRAKE. Confirm that
    start_task() creates a FreeRTOS task.
    ============================================================
*/
void test_robot_controller_initialization(Robot_Controller &r_controller) {
    // Confirm the constructor initialized.
    TEST_ASSERT_EQUAL(1, xQueueCreate_fake.call_count);
    TEST_ASSERT_EQUAL(10, xQueueCreate_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL(Robot_State::BRAKE, r_controller.get_robot_state());

    // Confirm robot controller task initialized.
    r_controller.start_task();
    TEST_ASSERT_EQUAL(1, xTaskCreate_fake.call_count);
    TEST_ASSERT_EQUAL_STRING("Robot_Controller_Task", xTaskCreate_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(4096, xTaskCreate_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(&r_controller, xTaskCreate_fake.arg3_history[0]);
    TEST_ASSERT_EQUAL(5, xTaskCreate_fake.arg4_history[0]);
    TEST_ASSERT_EQUAL(nullptr, xTaskCreate_fake.arg5_history[0]);

    return;
}
//  ============================================================



/*
    ============================================================
    Test that the monitoring logic inside the
    monitor_active_command() and process_new_command().
    ============================================================
*/
void test_command_monitoring(
        Robot_Controller &r_controller, Drive_Train &d_train,
        Robot_State state, uint32_t speed, uint32_t duration
    ) {
        // Set initial time to 0 ticks.
        xTaskGetTickCount_fake.return_val = 0;

        // Send the command to the queue.
        r_controller.send_command(state, speed, duration);
        r_controller.monitor_active_command();
        r_controller.process_new_command();
        TEST_ASSERT_EQUAL(state, r_controller.get_robot_state());

        // Simulate 500ms passing.
        xTaskGetTickCount_fake.return_val = 500 / portTICK_PERIOD_MS;
        r_controller.monitor_active_command();
        r_controller.process_new_command();
        TEST_ASSERT_EQUAL(state, r_controller.get_robot_state());

        // Simulate duration_ms + 500ms passing.
        xTaskGetTickCount_fake.return_val = (duration + 500) / portTICK_PERIOD_MS;
        int initial_ledc_calls = ledc_set_duty_fake.call_count;
        r_controller.monitor_active_command();
        r_controller.process_new_command();
        TEST_ASSERT_EQUAL(Robot_State::BRAKE, r_controller.get_robot_state());

        // If the robot is already braking, then no transition to BRAKE occurs.
        if (state != Robot_State::BRAKE) {
            TEST_ASSERT_EQUAL(initial_ledc_calls + 8, ledc_set_duty_fake.call_count);
        }
        else {
            TEST_ASSERT_EQUAL(initial_ledc_calls, ledc_set_duty_fake.call_count);
        }

        return;
}
//  ============================================================



/*
    ============================================================
    Test the routing logic in monitor_active_command() and
    process_new_command().
    ============================================================
*/
void test_command_routing(
        Robot_Controller &r_controller, Drive_Train &d_train,
        Robot_State state, uint32_t speed, uint32_t duration,
        int send_expected_call_index, int receive_expected_call_index
    ) {
        // Send the command to the queue.
        r_controller.send_command(state, speed, duration);

        TEST_ASSERT_EQUAL(send_expected_call_index, xQueueSend_fake.call_count);
        TEST_ASSERT_EQUAL(state, sent_command.target_state);
        TEST_ASSERT_EQUAL(speed, sent_command.speed);
        TEST_ASSERT_EQUAL(duration, sent_command.duration_ms);
        TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueSend_fake.arg2_history[send_expected_call_index - 1]);

        // Set random encoder pulses to prove they get reset by the controller.
        d_train.get_left_encoder().pulse_count = send_expected_call_index * 5;
        d_train.get_right_encoder().pulse_count = send_expected_call_index * 5;

        // Capture LEDC hardware call count.
        int initial_ledc_count = ledc_set_duty_fake.call_count;

        xTaskGetTickCount_fake.return_val += 10000 / portTICK_PERIOD_MS;

        // Receive and process the sent command.
        r_controller.monitor_active_command();
        r_controller.process_new_command();
        Robot_Command received_command = r_controller.get_active_command();

        TEST_ASSERT_EQUAL(receive_expected_call_index, xQueueReceive_fake.call_count);
        TEST_ASSERT_EQUAL(state, received_command.target_state);
        TEST_ASSERT_EQUAL(speed, received_command.speed);
        TEST_ASSERT_EQUAL(duration, received_command.duration_ms);
        TEST_ASSERT_EQUAL(pdMS_TO_TICKS(10), xQueueReceive_fake.arg2_history[receive_expected_call_index - 1]);

        // Confirm the controller updated its internal state and triggered the hardware.
        TEST_ASSERT_EQUAL(state, r_controller.get_robot_state());
        TEST_ASSERT_EQUAL(initial_ledc_count + 8, ledc_set_duty_fake.call_count);

        return;
}
//  ============================================================



/*
    ============================================================
    Confirm that that emergency_stop() clears the queue, sends a
    BRAKE command to the front of the queue, and locks out new
    commands from being sent to the queue.

    NOTE: There is no logic that lifts the lockout, so any
    commands that are sent after the emergency stop is called
    will be rejected. We will need to implement a method that
    lifts the lockout in the future to allow the robot to resume
    normal operations after an emergency stop.
    ============================================================
*/
void test_emergency_stop(Robot_Controller &r_controller) {
    r_controller.emergency_stop();

    TEST_ASSERT_EQUAL(1, xQueueReset_fake.call_count);
    TEST_ASSERT_EQUAL(1, xQueueSendToFront_fake.call_count);
    TEST_ASSERT_EQUAL(Robot_State::BRAKE, sent_emergency_command.target_state);
    TEST_ASSERT_EQUAL(0, sent_emergency_command.speed);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(10), xQueueSendToFront_fake.arg2_history[0]);

    // Attempt to send a command after the emergency stop.
    int initial_queue_sends = xQueueSend_fake.call_count;
    r_controller.send_command(Robot_State::MOVE_FORWARD, 255, 1000);

    // The call count should remain the same because the emergency lockout rejected the command from being sent to the queue.
    TEST_ASSERT_EQUAL(initial_queue_sends, xQueueSend_fake.call_count);

    return;
}
//  ============================================================



/*
    ============================================================
    Test the robot controller class and its functions.
    ============================================================
*/
void test_robot_controller_functions(void) {
    // Create the drive train and robot controller objects.
    Drive_Train d_train = create_drive_train_1();
    Robot_Controller r_controller(d_train);

    // Confirm that the robot controller is initialized and its task is created.
    test_robot_controller_initialization(r_controller);

    // Confirm the 5 movement commands are monitored.
    test_command_monitoring(r_controller, d_train, Robot_State::MOVE_FORWARD, 255, 1000);
    test_command_monitoring(r_controller, d_train, Robot_State::MOVE_BACKWARD, 225, 2500);
    test_command_monitoring(r_controller, d_train, Robot_State::TURN_LEFT, 210, 1234);
    test_command_monitoring(r_controller, d_train, Robot_State::TURN_RIGHT, 200, 4321);
    test_command_monitoring(r_controller, d_train, Robot_State::BRAKE, 0, 1000);

    // Confirm the 5 movement commands were sent and received.
    // Ensure the call indexes includes the counts of previous executed commands.
    test_command_routing(r_controller, d_train, Robot_State::MOVE_FORWARD, 255, 1000, 6, 11);
    test_command_routing(r_controller, d_train, Robot_State::MOVE_BACKWARD, 225, 2500, 7, 12);
    test_command_routing(r_controller, d_train, Robot_State::TURN_LEFT, 210, 1234, 8, 13);
    test_command_routing(r_controller, d_train, Robot_State::TURN_RIGHT, 200, 4321, 9, 14);
    test_command_routing(r_controller, d_train, Robot_State::BRAKE, 0, 1000, 10, 15);

    // Corfirm the emergency stop works and locks out new commands from being sent to the queue.
    test_emergency_stop(r_controller);

    return;
}
//  ============================================================