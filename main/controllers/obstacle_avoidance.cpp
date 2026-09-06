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
        loop_tick();
    }

    return;
}
//  ============================================================


/*
    ============================================================
    This tick function gets executed for every iteration in the
    infinite while loop above because we want to test the logic
    without having to access avoidance_loop() and get stuck in
    that loop.
    ============================================================
*/
void Obstacle_Avoidance::loop_tick() {
    // Reset the current calculated distance and state.
    double current_distance = 10000.0;   // Set it to an arbitrarily high number to avoid creating bugs.

    Robot_Controller &controller = get_controller();
    Robot_State current_state = controller.get_robot_state();

    if ((stopped_recently) && (current_state != Robot_State::BRAKE)) {
        ESP_LOGI(
            TAG,
            "Current movement is [%s]. Using [%s] ultrasonic sensor.",
            controller.state_strings[current_state],
            sensor_strings[current_state]
        );
    }
    else if ((!stopped_recently) && (current_state == Robot_State::BRAKE)) {
        ESP_LOGI(
            TAG,
            "Current movement is [%s]. Not using any ultrasonic sensors.",
            controller.state_strings[current_state]
        );
    }

    // Determine if the robot needs to perform an emergency stop upon detected object.
    current_distance = measure_object_distance(current_state, current_distance);
    determine_emergency_stop(current_distance);

    // Add a 50ms delay to give signal time to trigger and come back to sensor.
    vTaskDelay(pdMS_TO_TICKS(50));
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
        case Robot_State::MOVE_FORWARD: {
            Ultrasonic& front_sensor = get_ultrasonic_sensor("FRONT");
            front_sensor.measure_distance();
            current_distance = front_sensor.get_distance();
            stopped_recently = false;
            break;
        }

        case Robot_State::MOVE_BACKWARD: {
            Ultrasonic& back_sensor = get_ultrasonic_sensor("BACK");
            back_sensor.measure_distance();
            current_distance = back_sensor.get_distance();
            stopped_recently = false;
            break;
        }

        case Robot_State::TURN_LEFT: {
            Ultrasonic& left_sensor = get_ultrasonic_sensor("LEFT");
            left_sensor.measure_distance();
            current_distance = left_sensor.get_distance();
            stopped_recently = false;
            break;
        }

        case Robot_State::TURN_RIGHT: {
            Ultrasonic& right_sensor = get_ultrasonic_sensor("RIGHT");
            right_sensor.measure_distance();
            current_distance = right_sensor.get_distance();
            stopped_recently = false;
            break;
        }

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
    Robot_Controller &controller = get_controller();

    // Perform emergency stop if an ultrasonic sensor detects an object past the distance threshold.
    if (current_distance < get_distance_threshold()) {
        if (!stopped_recently) {
            ESP_LOGI(
                TAG_1,
                "Object is detected at [%0.4fmm]. Performing emergency stop.",
                current_distance
            );
        }

        controller.emergency_stop();
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Retrieve robot controller object.
    ============================================================
*/
Robot_Controller& Obstacle_Avoidance::get_controller() {
    return *(config.controller);
}
//  ============================================================


/*
    ============================================================
    Retrieve ultrasonic sensor object.
    ============================================================
*/
Ultrasonic& Obstacle_Avoidance::get_ultrasonic_sensor(const char* sensor) {
    if (sensor == sensor_strings[0]) {
        return *(config.front_sensor);
    }
    else if (sensor == sensor_strings[1]) {
        return *(config.back_sensor);
    }
    else if (sensor == sensor_strings[2]) {
        return *(config.left_sensor);
    }
    else if (sensor == sensor_strings[3]) {
        return *(config.right_sensor);
    }
    else {
        ESP_LOGE(TAG, "Error: Couldn't retrieve ultrasonic sensor with the provided name. Exiting program to prevent bugs/undefined behavior.");
        return *(config.front_sensor);  // This will stop the compiler from complaining about not returning object.
    }
}
//  ============================================================


/*
    ============================================================
    Retrieve distance threshold.
    ============================================================
*/
double Obstacle_Avoidance::get_distance_threshold() {
    return distance_threshold;
}
//  ============================================================