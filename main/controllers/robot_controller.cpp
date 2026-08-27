/*
    This file fleshes out the Robot_Controller class logic defined in robot_controller.hpp.
*/

/*
    ============================================================
    Include the following header files to access their
    class's methods and variables
    ============================================================
*/
#include <../include/controllers/robot_controller.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Robot_Controller class that initializes
    the robot's controller with user-defined values.
    ============================================================
*/
Robot_Controller::Robot_Controller(Drive_Train &train) : 
    drive_train(train),
    current_state(Robot_State::IDLE),
    command_queue(xQueueCreate(10, sizeof(Robot_Command)))  // Queue holds up to 10 commands at a time.
    {
        ESP_LOGI(CONTROLLER_NAME, "Initialized controller with a command queue size of 10.");
}
//  ============================================================


/*
    ============================================================
    Gets the robot's current state.
    ============================================================
*/
Robot_State Robot_Controller::get_robot_state() {
    return current_state;
}
//  ============================================================


/*
    ============================================================
    Stop the robot from moving.
    ============================================================
*/
void Robot_Controller::emergency_stop() {
    Robot_Command command = {
        Robot_State::BRAKE, 0, 1000
    };

    // Clear the queue to prevent the robot from executing the remaining movements and
    // potentially collide into the object.
    xQueueReset(command_queue);

    // Send the emergency stop command to the front of the queue.
    if (xQueueSendToFront(command_queue, &command, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGI("Emergency_Stop", "Failed to send emergency stop to the front of the queue.");
    }

    emergency_lockout = true;
    
    return;
}
//  ============================================================


/*
    ============================================================
    Spawn the task with a stack size and priority ranking.
    ============================================================
*/
void Robot_Controller::start_task() {
    xTaskCreate(task_queue, "Robot_Task", 4096, this, 5, nullptr);
    return;
}
//  ============================================================


/*
    ============================================================
    Configures the movement command and sends it to the
    command queue for execution.
    ============================================================
*/
void Robot_Controller::send_command(Robot_State state, uint32_t speed, uint32_t duration_ms) {
    // Don't send the command to the queue if the robot initiated an emergency stop.
    if (emergency_lockout == true) {
        ESP_LOGI("Robot_Controller", "Command is rejected because the robot performed an emergency stop.");
        return;
    }

    // Configure the command by defining the movement, how fast the robot needs to move,
    // and how long the robot needs to perform the movement.
    Robot_Command command = {
        state, speed, duration_ms
    };

    xQueueSend(command_queue, &command, portMAX_DELAY);
    return;
}
//  ============================================================


/*
    ============================================================
    Instantiates the controller to access member variables.
    ============================================================
*/
void Robot_Controller::task_queue(void *arg) {
    Robot_Controller *instance = static_cast<Robot_Controller*>(arg);
    instance->control_loop();
    return;
}
//  ============================================================


/*
    ============================================================
    An infinite loop that performs various movements based on
    the commands stored inside a queue. If no commands are in
    the queue, then the robot enters into an IDLE state.
    ============================================================
*/
void Robot_Controller::control_loop() {
    // Initially set the current command to be IDLE.
    Robot_Command current_command = {
        Robot_State::IDLE, 0, 0
    };

    TickType_t state_start_time = xTaskGetTickCount();

    while (true) {
        // Calculate wait time based on the state from the current command.
        TickType_t wait_time = (current_state == Robot_State::IDLE) ? portMAX_DELAY : pdMS_TO_TICKS(10);

        // Execute the command at the head of the queue.
        if (xQueueReceive(command_queue, &current_command, wait_time) == pdTRUE) {
            // Reset the wheel encoder sensors' pulse counts before executing
            // the new movement.
            drive_train.get_left_encoder().reset_count();
            drive_train.get_right_encoder().reset_count();

            // Configure the robot's movement and start the timer.
            current_state = current_command.target_state;
            drive_train.change_speed(current_command.speed, current_command.speed);
            state_start_time = xTaskGetTickCount();

            // Perform movement based on the state.
            switch (current_state) {
                case Robot_State::MOVE_FORWARD:
                    ESP_LOGI(
                        CONTROLLER_NAME,
                        "Current movement is FORWARD at speeds %lu and %lu.",
                        current_command.speed, current_command.speed
                    );
                    drive_train.move_forward();
                    break;

                case Robot_State::MOVE_BACKWARD:
                    ESP_LOGI(
                        CONTROLLER_NAME,
                        "Current movement is BACKWARD at speeds %lu and %lu.",
                        current_command.speed, current_command.speed
                    );
                    drive_train.move_backward();
                    break;

                case Robot_State::TURN_LEFT:
                    ESP_LOGI(
                        CONTROLLER_NAME,
                        "Current movement is LEFT at speeds %lu and %lu.",
                        current_command.speed, current_command.speed
                    );
                    drive_train.turn_left();
                    break;

                case Robot_State::TURN_RIGHT:
                    ESP_LOGI(
                        CONTROLLER_NAME,
                        "Current movement is RIGHT at speeds %lu and %lu.",
                        current_command.speed, current_command.speed
                    );
                    drive_train.turn_right();
                    break;

                case Robot_State::BRAKE:
                case Robot_State::IDLE:
                    ESP_LOGI(
                        CONTROLLER_NAME,
                        "Current movement is STOP at speeds %lu and %lu.",
                        current_command.speed, current_command.speed
                    );
                    drive_train.brake_all();
                    break;
            }
        }

        // Calculate the current time to determine if it reached the duration so that we can stop
        // the robot from performing the movement.
        if ((current_state != Robot_State::IDLE) && (current_command.duration_ms > 0)) {
            uint32_t elapsed_ms = (xTaskGetTickCount() - state_start_time) * portTICK_PERIOD_MS;

            if (elapsed_ms >= current_command.duration_ms) {
                ESP_LOGI(
                        CONTROLLER_NAME,
                        "The duration for the current movement (%lu ms) is reached at %lu ms. Stopping movement.",
                        current_command.duration_ms, elapsed_ms
                    );
                drive_train.brake_all();
                current_state = Robot_State::IDLE;
            }
        }
    }

    return;
}
//  ============================================================