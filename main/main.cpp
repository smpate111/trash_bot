/*
    ============================================================
    Libraries and headers used in this project.
    ============================================================
*/
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <led_strip.h>

#include <sdkconfig.h>
#include <stdio.h>
#include <string>

#include "controllers/drive_train.hpp"
//#include <../include/controllers/robot_controller.hpp>
//#include <../include/controllers/obstacle_avoidance.hpp>

#include "motors/motor.hpp"
#include "motors/motor_driver.hpp"

#include "sensors/wheel_encoder.hpp"
//#include <../include/sensors/ultrasonic.hpp>
//  ============================================================



/*
    ============================================================
    This task handles the robot's navigation by using the
    robot controller to command the robot to move 4 different
    ways.
    ============================================================
*/
/*
void Navigation_Task(void *arg) {
    Robot_Controller *controller = static_cast<Robot_Controller*>(arg);
    //vTaskDelay(pdMS_TO_TICKS(2000));

    controller->send_command(Robot_State::BRAKE, 0, 2000);

    controller->send_command(Robot_State::MOVE_FORWARD, 255, 2000);
    //vTaskDelay(pdMS_TO_TICKS(5000));
    controller->send_command(Robot_State::BRAKE, 0, 2000);

    controller->send_command(Robot_State::MOVE_BACKWARD, 255, 2000);
    //vTaskDelay(pdMS_TO_TICKS(5000));
    controller->send_command(Robot_State::BRAKE, 0, 2000);

    controller->send_command(Robot_State::TURN_LEFT, 255, 2000);
    //vTaskDelay(pdMS_TO_TICKS(5000));
    controller->send_command(Robot_State::BRAKE, 0, 2000);

    controller->send_command(Robot_State::TURN_RIGHT, 255, 2000);
    //vTaskDelay(pdMS_TO_TICKS(5000));
    controller->send_command(Robot_State::BRAKE, 0, 2000);

    vTaskDelete(NULL);
    return;
}
*/
//  ============================================================



/*
    ============================================================
    Converts C to C++.
    ============================================================
*/
extern "C" {
    void app_main(void);
}
//  ============================================================



/*
    ============================================================
    Main function that runs the code.
    ============================================================
*/
void app_main(void) {
    /*
    led_strip_handle_t led_strip;
    // Configure the NeoPixel LED strip configuration.
    led_strip_config_t strip = {};
    strip.strip_gpio_num = GPIO_NUM_38;
    strip.max_leds = 1;                 // The board has 1 LED
    strip.led_model = LED_MODEL_WS2812; // The onboard LED model

    // Configure the RMT peripheral backend driver
    led_strip_rmt_config_t rmt = {};
    rmt.clk_src = RMT_CLK_SRC_DEFAULT;

    // Initialize the led strip driver
    led_strip_new_rmt_device(&strip, &rmt, &led_strip);

    led_strip_clear(led_strip);
    */

    // Configure the PWM timer to specify the PWM signal's frequency and duty cycle resolution.
    ledc_timer_config_t timer_config = {};

    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;      // Have the hardware timer operate in low-speed.
    timer_config.duty_resolution = LEDC_TIMER_8_BIT;    // Set the PWM duty cycle to 8 bits.
    timer_config.timer_num = LEDC_TIMER_0;              // Pick the first hardware timer index to generate the PWM frequency and resolution.
    timer_config.freq_hz = 1000;                        // Configure the target frequency to 1000 Hz.
    timer_config.clk_cfg = LEDC_AUTO_CLK;               // Automatically select the highest frequency clock based on the PWM frequency and resolution.

    ledc_timer_config(&timer_config);



    // Configure the interrupt service routine.
    static const char* ISR_TAG = "Interrupt Service Routine";
    esp_err_t isr_error = gpio_install_isr_service(0);
    if ((isr_error != ESP_OK) && (isr_error != ESP_ERR_INVALID_STATE)) {
        //printf("Error installing ISR service.\n\n");
        ESP_LOGE(ISR_TAG, "Error installing ISR. Exiting.");
        return;
    }

    ESP_LOGI(ISR_TAG, "ISR initialized.");



    // Configure the motors by defining their pins and channels.
    Motor_Config fl_config = {"Front Left Motor", GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    Motor_Config fr_config = {"Front Right Motor", GPIO_NUM_8, GPIO_NUM_3, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    Motor_Config rl_config = {"Rear Left Motor", GPIO_NUM_6, GPIO_NUM_7, LEDC_CHANNEL_5, LEDC_CHANNEL_4};
    Motor_Config rr_config = {"Rear Right Motor", GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_7, LEDC_CHANNEL_6};

    // Initialize the motors.
    Motor fl_motor(fl_config);
    Motor fr_motor(fr_config);
    Motor rl_motor(rl_config);
    Motor rr_motor(rr_config);

    // Configure the motor drivers.
    Driver_Config fmd_config = {"Front Motor Driver", fl_motor, fr_motor};
    Driver_Config rmd_config = {"Rear Motor Driver", rl_motor, rr_motor};

    // Initialize the motor drivers.
    Motor_Driver fm_driver(fmd_config);
    Motor_Driver rm_driver(rmd_config);

    // Configure the wheel encoders.
    Encoder_Config lwe_config = {"Left Wheel Encoder", GPIO_NUM_1, 80.0, 20};
    Encoder_Config rwe_config = {"Right Wheel Encoder", GPIO_NUM_41, 80.0, 20};

    // Initialize the wheel encoders.
    Wheel_Encoder lw_encoder(lwe_config);
    Wheel_Encoder rw_encoder(rwe_config);

    // Configure the drive train.
    Train_Config dt_config = {"Drive Train", fm_driver, rm_driver, lw_encoder, rw_encoder};

    // Initialize the drive train.
    Drive_Train d_train(dt_config);

    /*
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int i = 0; i < 10; i++) {
        lw_encoder.reset_count();
        rw_encoder.reset_count();
        fm_driver.set_left_duty_cycle(255);
        fm_driver.set_right_duty_cycle(255);
        bm_driver.set_left_duty_cycle(255);
        bm_driver.set_right_duty_cycle(255);

        fm_driver.forward();
        bm_driver.forward();

        vTaskDelay(pdMS_TO_TICKS(2000));

        fm_driver.stop();
        bm_driver.stop();
        lw_encoder.reset_count();
        rw_encoder.reset_count();

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    */


    
    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.reset_encoder_counts();
    d_train.set_left_duty_cycle(255);
    d_train.set_right_duty_cycle(255);
    
    d_train.forward();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.stop();
    ESP_LOGI("Main", "Left Wheel Distance traveled: [%0.4fmm].", d_train.get_left_distance());
    ESP_LOGI("Main", "Right Wheel Distance traveled: [%0.4fmm].", d_train.get_right_distance());
    d_train.reset_encoder_counts();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.reset_encoder_counts();
    d_train.set_left_duty_cycle(215);
    d_train.set_right_duty_cycle(215);

    d_train.backward();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.stop();
    ESP_LOGI("Main", "Left Wheel Distance traveled: [%0.4fmm].", d_train.get_left_distance());
    ESP_LOGI("Main", "Right Wheel Distance traveled: [%0.4fmm].", d_train.get_right_distance());
    d_train.reset_encoder_counts();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.reset_encoder_counts();
    d_train.set_left_duty_cycle(190);
    d_train.set_right_duty_cycle(190);

    d_train.left_turn();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.stop();
    ESP_LOGI("Main", "Left Wheel Distance traveled: [%0.4fmm].", d_train.get_left_distance());
    ESP_LOGI("Main", "Right Wheel Distance traveled: [%0.4fmm].", d_train.get_right_distance());
    d_train.reset_encoder_counts();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.reset_encoder_counts();
    d_train.set_left_duty_cycle(155);
    d_train.set_right_duty_cycle(155);

    d_train.right_turn();

    vTaskDelay(pdMS_TO_TICKS(2000));

    d_train.stop();
    ESP_LOGI("Main", "Left Wheel Distance traveled: [%0.4fmm].", d_train.get_left_distance());
    ESP_LOGI("Main", "Right Wheel Distance traveled: [%0.4fmm].", d_train.get_right_distance());
    d_train.reset_encoder_counts();

    vTaskDelay(pdMS_TO_TICKS(2000));
    


    /*
    // Configure the motor drivers by defining which motors they are going to control.
    Driver_Config fd_config = {
        "Front Driver",
        fl_motor, fr_motor
    };

    Driver_Config bd_config = {
        "Back Driver",
        bl_motor, br_motor
    };

    // Initialize the motor drivers.
    Motor_Driver f_driver(fd_config);
    Motor_Driver b_driver(bd_config);



    // Configure the wheel encoder sensors by defining their pins.
    Encoder_Config le_config = {
        "Left Encoder", GPIO_NUM_1,
        80.0, 20
    };

    Encoder_Config re_config = {
        "Right Encoder", GPIO_NUM_41,
        80.0, 20
    };

    // Initialize the wheel encoder sensors.
    Wheel_Encoder l_encoder(le_config);
    Wheel_Encoder r_encoder(re_config);



    // Configure the drive train by defining which motor drivers and
    // wheel encoder sensors it is going to control.
    Train_Config t_config = {
        "Drive Train",
        f_driver, b_driver,
        &l_encoder, &r_encoder  // Pass the reference and not a copy because we want the original
                                // variables to be directly altered.
    };

    // Initialize the drive train.
    Drive_Train d_train(t_config);



    // Configure the ultrasonic sensors by defining their pins.
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

    // Initialize the ultrasonic sensors.
    Ultrasonic f_us(fus_config);
    Ultrasonic b_us(bus_config);
    Ultrasonic l_us(lus_config);
    Ultrasonic r_us(rus_config);

    // Configure the robot controller by passing the drive train.
    Robot_Controller controller(d_train);

    // Configure the obstacle avoidance by defining which ultrasonic sensors
    // and robot controller it is going to control.
    Avoid_Config a_config = {
        "Obstacle Avoidance",
        &f_us, &b_us, &l_us, &r_us,     // Pass the reference and not a copy because we want the original
        &controller                     // variables to be directly altered.
    };

    // Initialize the obstacle avoidance.
    Obstacle_Avoidance o_avoid(a_config);


    // Set a 2 second delay to give the program time to configure the settings.
    vTaskDelay(pdMS_TO_TICKS(2000));



    // Start the odometry task.
    d_train.start_task();
    
    // Start the robot controller task.
    controller.start_task();

    // Start the obstacle avoidance task.
    o_avoid.start_task();

    // Configure the navigation task by passing the robot controller.
    xTaskCreate(Navigation_Task, "Navigation_Task", 4096, &controller, 5, nullptr);

    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
    */
}
//  ============================================================