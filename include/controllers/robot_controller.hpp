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
    BRAKE,
    IDLE,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
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

        ~Robot_Controller() = default;

        void start_task();
        void send_command(Robot_State state, uint32_t speed, uint32_t duration_ms);
        void emergency_stop();
        
        Robot_State get_robot_state();

    // Set these variables to private to prevent access and modifications from outside the class.
    private:
        static constexpr const char* const CONTROLLER_NAME = "Robot Controller";

        Drive_Train &drive_train;
        Robot_State current_state;
        QueueHandle_t command_queue;

        bool emergency_lockout = false;

        static void task_queue(void *arg);

        void control_loop();
};
//  ============================================================

#endif