/*
    This file fleshes out the Obstacle_Avoidance class logic defined in obstacle_avoidance.hpp.
*/

/*
    ============================================================
    Include the following header files to access their
    class's methods and variables
    ============================================================
*/
#include <../include/controllers/obstacle_avoidance.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Obstacle_Avoidance class that
    initializes the sensors with user-defined values.
    ============================================================
*/
Obstacle_Avoidance::Obstacle_Avoidance(const Avoid_Config &avoid_setup) : config(avoid_setup) {
    ESP_LOGI(config.name.c_str(), "Initialized obstacle avoidance.");
}
//  ============================================================