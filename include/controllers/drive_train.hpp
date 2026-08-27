/*
    This file defines the Drive_Train class that is used to control 4
    motors, 2 wheel encoders, and be able to perform basic movements
    through 2 L298N motor drivers.
*/

#ifndef DRIVE_TRAIN_HPP_
#define DRIVE_TRAIN_HPP_

/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <../include/motors/motor_driver.hpp>
#include <../include/sensors/wheel_encoder.hpp>
//  ============================================================



/*
    ============================================================
    Struct that stores the user-defined configurations for the
    drive train.
    ============================================================
*/
struct Train_Config {
    std::string name;
    Motor_Driver Front_Driver;
    Motor_Driver Back_Driver;
    Wheel_Encoder *Left_Encoder;    // Must hold the original object.
    Wheel_Encoder *Right_Encoder;   // Must hold the original object.
};
//  ============================================================



/*
    ============================================================
    This class manages 4 motors' speed and direction.
    ============================================================
*/
class Drive_Train {
    // Set these methods to public to allow access and control from outside the class.
    public:
        Drive_Train(const Train_Config &train_setup);

        virtual ~Drive_Train() = default;

        virtual void change_speed(uint32_t left_speed, uint32_t right_speed);
        virtual void move_forward();
        virtual void move_backward();
        virtual void turn_left();
        virtual void turn_right();
        virtual void brake_all();

        Wheel_Encoder& get_left_encoder();
        Wheel_Encoder& get_right_encoder();

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        Train_Config config;
};
//  ============================================================

#endif