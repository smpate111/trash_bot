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
#include "motors/motor_driver.hpp"
#include "sensors/wheel_encoder.hpp"
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
    Motor_Driver Rear_Driver;
    Wheel_Encoder &Left_Encoder;    // Must reference the original object.
    Wheel_Encoder &Right_Encoder;   // Must reference the original object.
};
//  ============================================================



/*
    ============================================================
    Enum that stores the drive train's current state.
    ============================================================
*/
enum class Train_State {
    UNINITIALIZED,
    READY,
    FAULT
};
//  ============================================================



/*
    ============================================================
    Enum that stores the drive train's current command.
    ============================================================
*/
enum class Train_Command {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT_TURN,
    RIGHT_TURN
};
//  ============================================================



/*
    ============================================================
    This class manages the actuator output and direction of 2
    motor drivers and sensor output of 2 wheel encoders.
    ============================================================
*/
class Drive_Train {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Drive_Train(const Train_Config &train_setup);
        ~Drive_Train() = default;
        bool is_initialized() const;
        bool is_faulted() const;

        Train_Command get_train_command() const;
        Motor_Driver_Command get_front_driver_command() const;
        Motor_Driver_Command get_rear_driver_command() const;
        Motor_Command get_fd_left_motor_command() const;
        Motor_Command get_fd_right_motor_command() const;
        Motor_Command get_rd_left_motor_command() const;
        Motor_Command get_rd_right_motor_command() const;

        void set_left_duty_cycle(uint8_t left_duty);
        uint8_t get_left_duty_cycle() const;

        void set_right_duty_cycle(uint8_t right_duty);
        uint8_t get_right_duty_cycle() const;

        void forward();
        void backward();
        void left_turn();
        void right_turn();
        void stop();

        uint32_t get_left_pulse_count() const;
        uint32_t get_right_pulse_count() const;
        void reset_encoder_counts();

        double get_left_distance() const;
        double get_right_distance() const;

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        bool has_motor_driver_fault() const;
        bool has_encoder_fault() const;
        void enter_fault(const char* operation);

        Train_Config config;
        Train_State state = Train_State::UNINITIALIZED;
        Train_Command command = Train_Command::STOP;
        uint8_t current_left_duty = 0;
        uint8_t current_right_duty = 0;
};
//  ============================================================

#endif