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
    Motor_Config l_config = {
        "Left Motor",
        GPIO_NUM_11, GPIO_NUM_12,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1
    };

    Motor_Config r_config = {
        "Right Motor",
        GPIO_NUM_13, GPIO_NUM_13,
        LEDC_CHANNEL_2, LEDC_CHANNEL_3
    };

    Motor l_motor(l_config);
    Motor r_motor(r_config);

    // Create the motor driver object.
    Driver_Config config = {
        "Driver",
        l_motor, r_motor
    };

    return;
}