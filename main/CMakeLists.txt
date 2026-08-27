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