/*
    This file defines the Obstacle_Avoidance class used to control 4 ultrasonic sensors,
    turn each of them on based on the 4 movements, and have the robot stop when the sensors
    detect the robot is about to hit an obstacle.
*/

#ifndef OBSTACLE_AVOIDANCE_HPP_
#define OBSTACLE_AVOIDANCE_HPP_

/*
    ============================================================
    Define the class's dependencies.
    ============================================================
*/
#include <../include/controllers/robot_controller.hpp>
#include <../include/sensors/ultrasonic.hpp>
//  ============================================================



/*
    ============================================================
    Struct that stores the user-defined configurations for the
    obstacle avoidance.
    ============================================================
*/
struct Avoid_Config {
    std::string name;
    
    // Must hold the original objects.
    Ultrasonic *front_sensor;
    Ultrasonic *back_sensor;
    Ultrasonic *left_sensor;
    Ultrasonic *right_sensor;
    Robot_Controller *controller;
};
//  ============================================================



/*
    ============================================================
    This class manages 4 ultrasonic sensors.
    ============================================================
*/
class Obstacle_Avoidance {
    // Set these methods to public to allow access and control from outside the class.
    public:
        explicit Obstacle_Avoidance(const Avoid_Config &avoid_setup);
        virtual ~Obstacle_Avoidance() = default;

        void start_task();
        void loop_tick();

        double measure_object_distance(Robot_State current_state, double current_distance);
        void determine_emergency_stop(double current_distance);

        Robot_Controller& get_controller();
        Ultrasonic& get_ultrasonic_sensor(const char* sensor);
        double get_distance_threshold();

        Avoid_Config config;
        const char* sensor_strings[4] = {
            "FRONT",
            "BACK",
            "LEFT",
            "RIGHT"
        };

    // Set these to private to prvent access and modifications from outside the class.
    private:
        static void task_queue(void *arg);
        void avoidance_loop();

        const char* TAG = "Obstacle_Avoidance_Task";
        const char* TAG_1 = "Detected Object";
        double distance_threshold = 50.0;
        bool stopped_recently = true;
};
//  ============================================================

#endif