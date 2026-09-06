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

#include <test/mock_libraries/mock_hardware.hpp>

#include <test/test_motor_logic.cpp>
#include <test/test_motor_driver_logic.cpp>
#include <test/test_wheel_encoder_logic.cpp>
#include <test/test_ultrasonic_logic.cpp>

#include <test/test_drive_train_logic.cpp>
#include <test/test_robot_controller_logic.cpp>
#include <test/test_obstacle_avoidance_logic.cpp>
//  ============================================================


DEFINE_FFF_GLOBALS;


// Define the fake LEDC functions.
DEFINE_FAKE_VALUE_FUNC(esp_err_t, ledc_channel_config, const ledc_channel_config_t*);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, ledc_set_duty, ledc_mode_t, ledc_channel_t, uint32_t);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, ledc_update_duty, ledc_mode_t, ledc_channel_t);


// Define the fake GPIO functions.
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpio_config, const gpio_config_t*)
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpio_isr_handler_add, gpio_num_t, gpio_isr_t, void*);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpio_reset_pin, gpio_num_t);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpio_set_direction, gpio_num_t, gpio_mode_t);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpio_set_level, gpio_num_t, uint32_t);
DEFINE_FAKE_VALUE_FUNC(int, gpio_get_level, gpio_num_t);


// Define the fake ESP32 timer.
DEFINE_FAKE_VALUE_FUNC(int64_t, esp_timer_get_time);


// Define the fake FreeRTOS functions.
DEFINE_FAKE_VALUE_FUNC(QueueHandle_t, xQueueCreate, UBaseType_t, UBaseType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueSend, QueueHandle_t, const void*, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueSendToFront, QueueHandle_t, const void*, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueReceive, QueueHandle_t, void*, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xQueueReset, QueueHandle_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xTaskCreate, TaskFunction_t, const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*);
DEFINE_FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
DEFINE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);



/*
    ============================================================
    Setup the fake GPIO and LEDC functions by resetting them
    and their data prior to every test.
    ============================================================
*/
void setUp(void) {
    RESET_FAKE(ledc_channel_config);
    RESET_FAKE(ledc_set_duty);
    RESET_FAKE(ledc_update_duty);

    RESET_FAKE(gpio_config);
    RESET_FAKE(gpio_isr_handler_add);
    RESET_FAKE(gpio_reset_pin);
    RESET_FAKE(gpio_set_direction);
    RESET_FAKE(gpio_set_level);
    RESET_FAKE(gpio_get_level);

    RESET_FAKE(esp_timer_get_time);

    RESET_FAKE(xQueueCreate);
    RESET_FAKE(xQueueSend);
    RESET_FAKE(xQueueSendToFront);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueueReset);
    RESET_FAKE(xTaskCreate);
    RESET_FAKE(xTaskGetTickCount);
    RESET_FAKE(vTaskDelay);

    // Reset these custom fake functions to capture commands sent to the FreeRTOS queue.
    xQueueSend_fake.custom_fake = custom_xQueueSend;
    xQueueSendToFront_fake.custom_fake = custom_xQueueSendToFront;
    xQueueReceive_fake.custom_fake = custom_xQueueReceive;

    FFF_RESET_HISTORY();
}
//  ============================================================



/*
    ============================================================
    Tear down the functions after finishing the tests.
    ============================================================
*/
void tearDown(void) {}
//  ============================================================



/*
    ============================================================
    Setup the fake GPIO and LEDC functions by resetting them
    and their data prior to every test.
    ============================================================
*/
extern "C" void app_main(void) {
    UNITY_BEGIN();

    // Test the hardware.
    RUN_TEST(test_motor_functions);
    RUN_TEST(test_motor_driver_functions);
    RUN_TEST(test_wheel_encoder_functions);
    RUN_TEST(test_ultrasonic_functions);

    // Test the controllers.
    RUN_TEST(test_drive_train_functions);
    RUN_TEST(test_robot_controller_functions);
    RUN_TEST(test_obstacle_avoidance_functions);

    UNITY_END();

    // Exit the function once we performed all tests.
    exit(0);
}
//  ============================================================