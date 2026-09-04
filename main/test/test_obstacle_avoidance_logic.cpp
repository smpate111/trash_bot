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
Robot_Command sent_command;
Robot_Command sent_emergency_command;

// Define a global variable that stores the received command from the FreeRTOS queue.
Robot_Command received_command;



/*
    ============================================================
    Helper function that creates the drive train object.
    ============================================================
*/
Drive_Train create_drive_train_2() {
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
    Test the obstacle avoidance class and its functions.
    ============================================================
*/
void test_obstacle_avoidance_functions(void) {
    // Create the drive train and robot controller objects.
    Drive_Train d_train = create_drive_train_2();
    Robot_Controller r_controller(d_train);
    
    // Create the ultrasonic sensor objects.
    Ultrasonic_Config fus_config = {
        "Front Ultrasonic Sensor",
        GPIO_NUM_2, GPIO_NUM_42
    };

    Ultrasonic_Config bus_config = {
        "Back Ultrasonic Sensor",
        GPIO_NUM_5, GPIO_NUM_4
    };

    Ultrasonic_Config lus_config = {
        "Left Ultrasonic Sensor",
        GPIO_NUM_18, GPIO_NUM_17
    };

    Ultrasonic_Config rus_config = {
        "Right Ultrasonic Sensor",
        GPIO_NUM_9, GPIO_NUM_10
    };

    Ultrasonic f_us(fus_config);
    Ultrasonic b_us(bus_config);
    Ultrasonic l_us(lus_config);
    Ultrasonic r_us(rus_config);

    // Create the obstacle avoidance object.
    Avoid_Config a_config = {
        "Obstace Avoidance",
        &f_us, &b_us, &l_us, &r_us,
        &r_controller
    };

    Obstacle_Avoidance o_avoidance(a_config);

    /*
        ============================================================
        Test 1: Confirm that the robot controller object initialized
        correctly, created a 10 size command queue, and the robot's
        initial state is set to BRAKE.
        ============================================================
    */

    //  ============================================================


    /*
        ============================================================
        Test 2: 
        ============================================================
    */

    //  ============================================================


    /*
        ============================================================
        Test 3: 
        ============================================================
    */

    //  ============================================================

    return;
}