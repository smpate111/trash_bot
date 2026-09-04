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


/*
    ============================================================
    Spawn the task with a stack size and priority ranking.
    ============================================================
*/
void Obstacle_Avoidance::start_task() {
    xTaskCreate(task_queue, "Obstacle_Avoidance_Task", 4096, this, 5, nullptr);
    return;
}
//  ============================================================


/*
    ============================================================
    Instantiates the controller to access member variables.
    ============================================================
*/
void Obstacle_Avoidance::task_queue(void *arg) {
    Obstacle_Avoidance *instance = static_cast<Obstacle_Avoidance*>(arg);
    instance->avoidance_loop();
    return;
}
//  ============================================================


/*
    ============================================================
    An infinite loop that processes the command queue and
    executes the robot's movement based on the stored commands.
    ============================================================
*/
void Obstacle_Avoidance::avoidance_loop() {
    while (true) {
        // Reset the current calculated distance and state.
        double current_distance = 10000.0;   // Set it to an arbitrarily high number to avoid creating bugs.
        Robot_State current_state = config.controller->get_robot_state();

        if ((stopped_recently == true) && (current_state != Robot_State::BRAKE)) {
            ESP_LOGI(
                TAG,
                "Current movement is [%s]. Using [%s] ultrasonic sensor.",
                config.controller->state_strings[current_state],
                sensor_strings[current_state]
            );
        }
        else if ((stopped_recently == false) && (current_state == Robot_State::BRAKE)) {
            ESP_LOGI(
                TAG,
                "Current movement is [%s]. Not using any ultrasonic sensors.",
                config.controller->state_strings[current_state]
            );
        }

        // Determine if the robot needs to perform an emergency stop upon detected object.
        current_distance = measure_object_distance(current_state, current_distance);
        determine_emergency_stop(current_distance);

        // Add a 50ms delay to give signal time to trigger and come back to sensor.
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Use an ultrasonic sensor based on the robot's current state
    to calculate the object's distance from that sensor.
    ============================================================
*/
double Obstacle_Avoidance::measure_object_distance(Robot_State current_state, double current_distance) {
    switch (current_state) {
        case Robot_State::MOVE_FORWARD:
            config.front_sensor->measure_distance();
            current_distance = config.front_sensor->distance;
            stopped_recently = false;
            break;

        case Robot_State::MOVE_BACKWARD:
            config.back_sensor->measure_distance();
            current_distance = config.back_sensor->distance;
            stopped_recently = false;
            break;

        case Robot_State::TURN_LEFT:
            config.left_sensor->measure_distance();
            current_distance = config.left_sensor->distance;
            stopped_recently = false;
            break;

        case Robot_State::TURN_RIGHT:
            config.right_sensor->measure_distance();
            current_distance = config.right_sensor->distance;
            stopped_recently = false;
            break;

        case Robot_State::BRAKE:
            stopped_recently = true;
            break;
    }

    return current_distance;
}
//  ============================================================


/*
    ============================================================
    Determine if the robot needs to trigger an emergency stop if
    an ultrasonic sensor detects an object past the distance
    threshold.
    ============================================================
*/
void Obstacle_Avoidance::determine_emergency_stop(double current_distance) {
    // Perform emergency stop if an ultrasonic sensor detects an object past the distance threshold.
    if (current_distance < distance_threshold) {
        if (stopped_recently == false) {
            ESP_LOGI(
                TAG_1,
                "Object is detected at [%0.4fmm]. Performing emergency stop.",
                current_distance
            );
        }

        config.controller->emergency_stop();
    }

    return;
}
//  ============================================================