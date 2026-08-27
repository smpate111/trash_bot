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

        double distance_threshold = 50.0;
        Avoid_Config config;
};
//  ============================================================

#endif