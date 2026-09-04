/*
    This file defines the Robot_Controller class used to manage controlling
    the movements of the robot car.
*/

#ifndef ROBOT_CONTROLLER_HPP_
#define ROBOT_CONTROLLER_HPP_

/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <../include/controllers/drive_train.hpp>
//  ============================================================



/*
    ============================================================
    Enum that defines all states for the state machine.
    ============================================================
*/
enum Robot_State {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    BRAKE
};
//  ============================================================



/*
    ============================================================
    Struct that stores the command's data packet to the
    FreeRTOS queue.
    ============================================================
*/
struct Robot_Command {
    Robot_State target_state;
    uint32_t speed;
    uint32_t duration_ms;
};
//  ============================================================



/*
    ============================================================
    This class manages controlling the car.
    ============================================================
*/
class Robot_Controller {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Robot_Controller(Drive_Train &train);
        virtual ~Robot_Controller() = default;

        void start_task();

        void loop_tick();
        void process_new_command();
        void monitor_active_command();

        BaseType_t received_new_command(Robot_Command &command);
        void set_active_command(const Robot_Command &command);
        Robot_Command get_active_command();
        void execute_active_command();

        Robot_State get_robot_state();
        void set_robot_state(const Robot_State &new_state);

        void set_robot_speed(const uint32_t &speed);
        
        TickType_t get_current_time();
        void start_timer();
        TickType_t get_start_time();
        
        void send_command(Robot_State state, uint32_t speed, uint32_t duration_ms);
        void emergency_stop();
        
        const char* state_strings[5] = {
            "MOVE_FORWARD",
            "MOVE_BACKWARD",
            "TURN_LEFT",
            "TURN_RIGHT",
            "BRAKE"
        };

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        static void task_queue(void *arg);
        void control_loop();

        const char* TAG = "Robot_Controller_Task";
        const char* TAG_1 = "Emergency_Stop";

        Drive_Train &drive_train;
        Robot_State current_state;
        QueueHandle_t command_queue;

        TickType_t state_start_time = 0;

        Robot_Command active_command = {
            Robot_State::BRAKE,
            0,
            0
        };

        bool emergency_lockout = false;

        bool new_command = true;
};
//  ============================================================

#endif