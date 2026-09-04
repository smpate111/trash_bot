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
    current_state(Robot_State::BRAKE),
    command_queue(xQueueCreate(10, sizeof(Robot_Command)))  // Queue holds up to 10 commands at a time.
    {
        ESP_LOGI(TAG, "Initialized robot controller with a command queue size of 10.");
}
//  ============================================================


/*
    ============================================================
    Spawn the task with a stack size and priority ranking.
    ============================================================
*/
void Robot_Controller::start_task() {
    xTaskCreate(task_queue, "Robot_Controller_Task", 4096, this, 5, nullptr);
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
    An infinite loop that processes the command queue and
    executes the robot's movement based on the stored commands.
    ============================================================
*/
void Robot_Controller::control_loop() {
    start_timer();

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
    without having to access control_loop() and get stuck in
    that loop.
    ============================================================
*/
void Robot_Controller::loop_tick() {
    monitor_active_command();
    process_new_command();
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
}
//  ============================================================


/*
    ============================================================
    Monitors the command that is in the front of the queue.
    ============================================================
*/
void Robot_Controller::monitor_active_command() {
    // Calculate the command's elapsed time to determine if the robot should leave its current state.
    uint32_t elapsed_ms = (get_current_time() - get_start_time()) * portTICK_PERIOD_MS;

    Robot_State state = get_robot_state();
    Robot_Command command = get_active_command();

    // Stop executing the command when it reaches its duration.
    if (elapsed_ms >= command.duration_ms) {
        // Print log once after it reaches its duration.
        if (new_command == true) {
            ESP_LOGI(
                TAG,
                "Reached the current movement's [%s] duration. Stopping the execution of this movement.",
                state_strings[state]
            );

            new_command = false;
        }
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Checks and processes a new command that is in the queue.
    ============================================================
*/
void Robot_Controller::process_new_command() {
    // Calculate the command's elapsed time to determine if the robot should leave its current state.
    uint32_t elapsed_ms = (get_current_time() - get_start_time()) * portTICK_PERIOD_MS;

    // Execute the next command after the current command reached its duration.
    if (elapsed_ms >= get_active_command().duration_ms) {
        Robot_Command incoming_command;
        BaseType_t command_received = received_new_command(incoming_command);

        // If a command was received in the queue, then execute it.
        if (command_received == pdTRUE) {
            set_active_command(incoming_command);
            execute_active_command();
        }
        // If a command was not received in the queue, then check that the robot's current state
        // is any state besides BRAKE to stop the robot from moving.
        else if (get_robot_state() != Robot_State::BRAKE) {
            Robot_Command brake = {
                Robot_State::BRAKE,
                0,
                1000
            };

            set_active_command(brake);
            execute_active_command();
        }
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Check if there are any new commands in the queue for 10ms.
    ============================================================
*/
BaseType_t Robot_Controller::received_new_command(Robot_Command &command) {
    return xQueueReceive(command_queue, &command, pdMS_TO_TICKS(10));
}
//  ============================================================


/*
    ============================================================
    Performs the robot's movement based on the command stored
    in the front of the queue.
    ============================================================
*/
void Robot_Controller::execute_active_command() {
    Robot_Command command = get_active_command();

    // Reset the wheel encoders' pulse counts before executing the new command.
    drive_train.get_left_encoder().reset_count();
    drive_train.get_right_encoder().reset_count();

    // Update the robot's movement with the new command and reset the timer.
    set_robot_state(command.target_state);
    drive_train.change_speed(command.speed, command.speed);
    start_timer();

    Robot_State state = get_robot_state();

    new_command = true;
    ESP_LOGI(
        TAG,
        "Current movement is [%s] with speeds -- Left wheel(s): [%lu], Right Wheel(s): [%lu].",
        state_strings[state],
        command.speed,
        command.speed
    );

    // Perform movement based on the state.
    switch (state) {
        case Robot_State::MOVE_FORWARD:
            drive_train.move_forward();
            break;

        case Robot_State::MOVE_BACKWARD:
            drive_train.move_backward();
            break;

        case Robot_State::TURN_LEFT:
            drive_train.turn_left();
            break;

        case Robot_State::TURN_RIGHT:
            drive_train.turn_right();
            break;

        case Robot_State::BRAKE:
            drive_train.brake_all();
            break;
    }

    return;
}
//  ============================================================


/*
    ============================================================
    Updates the active command.
    ============================================================
*/
void Robot_Controller::set_active_command(const Robot_Command &command) {
    active_command = command;
    return;
}
//  ============================================================


/*
    ============================================================
    Retrieves the active command.
    ============================================================
*/
Robot_Command Robot_Controller::get_active_command() {
    return active_command;
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
    Updates the robot's current state with a new state.
    ============================================================
*/
void Robot_Controller::set_robot_state(const Robot_State &new_state) {
    current_state = new_state;
    return;
}
//  ============================================================


/*
    ============================================================
    Gets the current time.
    ============================================================
*/
TickType_t Robot_Controller::get_current_time() {
    return xTaskGetTickCount();
}
//  ============================================================


/*
    ============================================================
    Starts the timer.
    ============================================================
*/
void Robot_Controller::start_timer() {
    state_start_time = get_current_time();
    return;
}
//  ============================================================


/*
    ============================================================
    Gets the command's start time.
    ============================================================
*/
TickType_t Robot_Controller::get_start_time() {
    return state_start_time;
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
        ESP_LOGI(TAG_1, "Command is rejected because the robot performed an emergency stop.");
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
    Stop the robot from moving.
    ============================================================
*/
void Robot_Controller::emergency_stop() {
    Robot_Command command = {
        Robot_State::BRAKE, 0, 1000
    };

    //set_active_command(command);

    // Clear the queue to prevent the robot from executing the remaining movements and
    // potentially collide into the object.
    xQueueReset(command_queue);

    // Send the emergency stop command to the front of the queue.
    if (xQueueSendToFront(command_queue, &command, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGI(TAG, "Failed to send emergency stop to the front of the queue.");
        return;
    }

    emergency_lockout = true;

    ESP_LOGI(
        TAG_1,
        "Emergency stop is activated. The robot has stopped and the command queue is cleared. New commands are rejected until the emergency lockout is lifted."
    );
    
    return;
}
//  ============================================================