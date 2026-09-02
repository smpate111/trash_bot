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

// Define a global variable that stores the received command from the FreeRTOS queue.
Robot_Command received_command;



/*
    ============================================================
    Helper function that creates the drive train object.
    ============================================================
*/
Drive_Train create_drive_train_1() {
    // Create the motor objects.
    static Motor_Config fl_config = {
        "Front Left Motor",
        GPIO_NUM_11, GPIO_NUM_12,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1
    };

    static Motor_Config fr_config = {
        "Front Right Motor",
        GPIO_NUM_13, GPIO_NUM_14,
        LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };

    static Motor_Config bl_config = {
        "Back Left Motor",
        GPIO_NUM_6, GPIO_NUM_7,
        LEDC_CHANNEL_5, LEDC_CHANNEL_4
    };

    static Motor_Config br_config = {
        "Back Right Motor",
        GPIO_NUM_15, GPIO_NUM_16,
        LEDC_CHANNEL_7, LEDC_CHANNEL_6
    };

    static Motor fl_motor(fl_config);
    static Motor fr_motor(fr_config);
    static Motor bl_motor(bl_config);
    static Motor br_motor(br_config);

    // Create the motor driver objects.
    static Driver_Config fd_config = {
        "Front Driver",
        fl_motor, fr_motor
    };

    static Driver_Config bd_config = {
        "Back Driver",
        bl_motor, br_motor
    };

    static Motor_Driver f_driver(fd_config);
    static Motor_Driver b_driver(bd_config);

    // Create the wheel encoder objects.
    static Encoder_Config le_config = {
        "Left Encoder", GPIO_NUM_1,
        80.0, 20
    };

    static Encoder_Config re_config = {
        "Right Encoder", GPIO_NUM_2,
        80.0, 20
    };

    static Wheel_Encoder l_encoder(le_config);
    static Wheel_Encoder r_encoder(re_config);

    // Configure the drive train object.
    static Train_Config t_config = {
        "Drive Train",
        f_driver, b_driver,
        &l_encoder, &r_encoder
    };

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
    }
    return pdTRUE;
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
    }
    return pdTRUE;
}
//  ============================================================



/*
    ============================================================
    A fake function used by the FreeRTOS queue that receives the
    command.
    ============================================================
*/
BaseType_t custom_xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks) {
    if (item != nullptr) {
        *(Robot_Command*)item = sent_command;
    }
    return pdTRUE;
}
//  ============================================================



/*
    ============================================================
    Test the robot controller class and its functions.
    ============================================================
*/
void test_robot_controller_functions(void) {
    // Create the drive train object.
    Drive_Train d_train = create_drive_train_1();
    

    /*
        ============================================================
        Test 1: Confirm that the robot controller's constructor
        initializes the object correctly, creates a command queue of
        size 10, and sets the robot's state to BRAKE.
        ============================================================
    */
    Robot_Controller r_controller(d_train);

    TEST_ASSERT_EQUAL(1, xQueueCreate_fake.call_count);
    TEST_ASSERT_EQUAL(10, xQueueCreate_fake.arg0_history[0]);

    TEST_ASSERT_EQUAL(Robot_State::BRAKE, r_controller.get_robot_state());

    //  ============================================================


    /*
        ============================================================
        Test 2: Confirm that start_task() creates a FreeRTOS task.
        ============================================================
    */
    r_controller.start_task();

    TEST_ASSERT_EQUAL(1, xTaskCreate_fake.call_count);
    TEST_ASSERT_EQUAL_STRING("Robot_Task", xTaskCreate_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL(4096, xTaskCreate_fake.arg2_history[0]);
    TEST_ASSERT_EQUAL(&r_controller, xTaskCreate_fake.arg3_history[0]);
    TEST_ASSERT_EQUAL(5, xTaskCreate_fake.arg4_history[0]);
    TEST_ASSERT_EQUAL(nullptr, xTaskCreate_fake.arg5_history[0]);

    //  ============================================================


    /*
        ============================================================
        Test 3: Verify that send_command() correctly sends commands
        to the FreeRTOS queue and that the queue receives the
        correct command data.
        
        NOTE: We will use customized fake functions that captures
        the command sent to the queue before it goes out of scope to
        verify the data is correct. In hardware, the sent command
        would not be stored to enforce thread safety.
        ============================================================
    */

    /*
        Test 3.1: Confirm that the forward command is sent to the
        queue and the queue receives it.
    */
    // Send the command to the queue.
    r_controller.send_command(Robot_State::MOVE_FORWARD, 255, 1000);

    TEST_ASSERT_EQUAL(1, xQueueSend_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::MOVE_FORWARD, sent_command.target_state);
    TEST_ASSERT_EQUAL(255, sent_command.speed);
    TEST_ASSERT_EQUAL(1000, sent_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueSend_fake.arg2_history[0]);

    // Receive the sent command.
    received_command = r_controller.execute_command(sent_command);

    TEST_ASSERT_EQUAL(1, xQueueReceive_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::MOVE_FORWARD, received_command.target_state);
    TEST_ASSERT_EQUAL(255, received_command.speed);
    TEST_ASSERT_EQUAL(1000, received_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueReceive_fake.arg2_history[0]);
    //  ============================================================


    /*
        Test 3.2: Confirm that the backward command is sent to the
        queue and the queue receives it.
    */
    // Send the command to the queue.
    r_controller.send_command(Robot_State::MOVE_BACKWARD, 225, 1500);

    TEST_ASSERT_EQUAL(2, xQueueSend_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::MOVE_BACKWARD, sent_command.target_state);
    TEST_ASSERT_EQUAL(225, sent_command.speed);
    TEST_ASSERT_EQUAL(1500, sent_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueSend_fake.arg2_history[1]);

    // Receive the sent command.
    received_command = r_controller.execute_command(sent_command);

    TEST_ASSERT_EQUAL(2, xQueueReceive_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::MOVE_BACKWARD, received_command.target_state);
    TEST_ASSERT_EQUAL(225, received_command.speed);
    TEST_ASSERT_EQUAL(1500, received_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueReceive_fake.arg2_history[1]);
    //  ============================================================


    /*
        Test 3.3: Confirm that the left command is sent to the
        queue and the queue receives it.
    */
    // Send the command to the queue.
    r_controller.send_command(Robot_State::TURN_LEFT, 210, 3000);

    TEST_ASSERT_EQUAL(3, xQueueSend_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::TURN_LEFT, sent_command.target_state);
    TEST_ASSERT_EQUAL(210, sent_command.speed);
    TEST_ASSERT_EQUAL(3000, sent_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueSend_fake.arg2_history[2]);

    // Receive the sent command.
    received_command = r_controller.execute_command(sent_command);

    TEST_ASSERT_EQUAL(3, xQueueReceive_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::TURN_LEFT, received_command.target_state);
    TEST_ASSERT_EQUAL(210, received_command.speed);
    TEST_ASSERT_EQUAL(3000, received_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueReceive_fake.arg2_history[2]);
    //  ============================================================


    /*
        Test 3.4: Confirm that the right command is sent to the
        queue and the queue receives it.
    */
    // Send the command to the queue.
    r_controller.send_command(Robot_State::TURN_RIGHT, 200, 2200);

    TEST_ASSERT_EQUAL(4, xQueueSend_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::TURN_RIGHT, sent_command.target_state);
    TEST_ASSERT_EQUAL(200, sent_command.speed);
    TEST_ASSERT_EQUAL(2200, sent_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueSend_fake.arg2_history[3]);

    // Receive the sent command.
    received_command = r_controller.execute_command(sent_command);

    TEST_ASSERT_EQUAL(4, xQueueReceive_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::TURN_RIGHT, received_command.target_state);
    TEST_ASSERT_EQUAL(200, received_command.speed);
    TEST_ASSERT_EQUAL(2200, received_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueReceive_fake.arg2_history[3]);
    //  ============================================================


    /*
        Test 3.5: Confirm that the brake command is sent to the
        queue and the queue receives it.
    */
    // Send the command to the queue.
    r_controller.send_command(Robot_State::BRAKE, 0, 4444);

    TEST_ASSERT_EQUAL(5, xQueueSend_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::BRAKE, sent_command.target_state);
    TEST_ASSERT_EQUAL(0, sent_command.speed);
    TEST_ASSERT_EQUAL(4444, sent_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueSend_fake.arg2_history[4]);

    // Receive the sent command.
    received_command = r_controller.execute_command(sent_command);

    TEST_ASSERT_EQUAL(5, xQueueReceive_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::BRAKE, received_command.target_state);
    TEST_ASSERT_EQUAL(0, received_command.speed);
    TEST_ASSERT_EQUAL(4444, received_command.duration_ms);

    TEST_ASSERT_EQUAL(portMAX_DELAY, xQueueReceive_fake.arg2_history[4]);
    //  ============================================================


    /*
        ============================================================
        Test 4: Confirm that that emergency_stop() clears the queue,
        sends a BRAKE command to the front of the queue, and locks
        out new commands from being sent to the queue.

        NOTE: There is no logic that lifts the lockout, so any
        commands that are sent after the emergency stop is called
        will be rejected. We will need to implement a method that
        lifts the lockout in the future to allow the robot to resume
        normal operations after an emergency stop.
        ============================================================
    */
    r_controller.emergency_stop();

    TEST_ASSERT_EQUAL(1, xQueueReset_fake.call_count);
    TEST_ASSERT_EQUAL(1, xQueueSendToFront_fake.call_count);

    TEST_ASSERT_EQUAL(Robot_State::BRAKE, sent_emergency_command.target_state);
    TEST_ASSERT_EQUAL(0, sent_emergency_command.speed);

    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(10), xQueueSendToFront_fake.arg2_history[0]);

    // Attempt to send a command after the emergency stop.
    r_controller.send_command(Robot_State::MOVE_FORWARD, 255, 1000);

    // The call count should remain the same because the emergency lockout rejected the command from being sent to the queue.
    TEST_ASSERT_EQUAL(5, xQueueSend_fake.call_count);
    //  ============================================================

    return;
}