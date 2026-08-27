/*
    This file defines the Motor_Driver class used to control 2 motors,
    perform basic movements, and adjust speeds using the L298N motor
    driver.
*/

#ifndef MOTOR_DRIVER_HPP_
#define MOTOR_DRIVER_HPP_

/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <../include/motors/motor.hpp>
//  ============================================================



/*
    ============================================================
    Struct that stores the user-defined configurations for the
    motor driver.
    ============================================================
*/
struct Driver_Config {
    std::string name;
    Motor Left_Motor;
    Motor Right_Motor;
};
//  ============================================================



/*
    ============================================================
    This class manages 2 motors' speed and direction.
    ============================================================
*/
class Motor_Driver {
    // Set these methods to public to allow access and control from outside the class.
    public:
        Motor_Driver(const Driver_Config &driver_setup);
        
        virtual ~Motor_Driver() = default;

        virtual void adjust_speed(uint32_t left_speed, uint32_t right_speed);
        virtual void forward();
        virtual void backward();
        virtual void left();
        virtual void right();
        virtual void brake();

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        Driver_Config config;
};
//  ============================================================

#endif