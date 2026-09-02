if (IDF_TARGET STREQUAL "linux")
    # LINUX UNIT TESTING BUILD
    idf_component_register(
        SRCS
            "controllers/drive_train.cpp"
            "controllers/obstacle_avoidance.cpp"
            "controllers/robot_controller.cpp"
            
            "motors/motor.cpp"
            "motors/motor_driver.cpp"

            "sensors/ultrasonic.cpp"
            "sensors/wheel_encoder.cpp"

            "test/test_main.cpp"
            "test/test_motor_logic.cpp"
            "test/test_motor_driver_logic.cpp"
            "test/test_ultrasonic_logic.cpp"
            "test/test_wheel_encoder_logic.cpp"

            "test/test_drive_train_logic.cpp"
            "test/test_robot_controller_logic.cpp"

        INCLUDE_DIRS
            "."
            "../include"
            "test/mock_libraries"

        REQUIRES
            unity
            esp_timer
    )
else()
    # ESP32S3 BUILD
    idf_component_register(
        SRCS
            "controllers/drive_train.cpp"
            "controllers/obstacle_avoidance.cpp"
            "controllers/robot_controller.cpp"
            
            "motors/motor.cpp"
            "motors/motor_driver.cpp"

            "sensors/ultrasonic.cpp"
            "sensors/wheel_encoder.cpp"
            
            "main.cpp"

        INCLUDE_DIRS
            "."
            "../include/"
        
        REQUIRES
            driver
            esp_driver_gpio
            esp_driver_ledc
            esp_timer
    )
endif()