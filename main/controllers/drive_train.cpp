/*
    This file fleshes out the Drive_Train class logic defined in drive_train.hpp.
*/

/*
    ============================================================
    Include the following header files to access their
    class's methods and variables
    ============================================================
*/
#include <../include/controllers/drive_train.hpp>
//  ============================================================



/*
    ============================================================
    Constructor for the Drive_Train class that initializes the
    motors with user-defined values.
    ============================================================
*/
Drive_Train::Drive_Train(const Train_Config &train_setup) : config(train_setup) {
    ESP_LOGI(config.name.c_str(), "Initialized drive train.");
}
//  ============================================================


/*
    ============================================================
    Spawn the task with a stack size and priority ranking.
    ============================================================
*/
void Drive_Train::start_task() {
    xTaskCreate(task_queue, "Odometry_Task", 4096, this, 4, nullptr);
    return;
}
//  ============================================================


/*
    ============================================================
    Instantiates the drive train to access member variables.
    ============================================================
*/
void Drive_Train::task_queue(void *arg) {
    Drive_Train *instance = static_cast<Drive_Train*>(arg);
    instance->odometry_loop();
    return;
}
//  ============================================================


/*
    ============================================================
    An infinite loop that processes the odometry task and
    executes the robot's movement.
    ============================================================
*/
void Drive_Train::odometry_loop() {
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
    without having to access odometry_loop() and get stuck in
    that loop.
    ============================================================
*/
void Drive_Train::loop_tick() {
    // Retrieve the pulse counts.
    Wheel_Encoder &l_encoder = get_left_encoder();
    Wheel_Encoder &r_encoder = get_right_encoder();
    uint32_t l_pulses = get_pulses(l_encoder);
    uint32_t r_pulses = get_pulses(r_encoder);
    set_l_distance(l_encoder.calculate_distance());
    set_r_distance(r_encoder.calculate_distance());
    
    // Record distance and pulse count while robot is moving.
    if ((get_last_l_pulses() != l_pulses) || (get_last_r_pulses() != r_pulses)) {
        ESP_LOGI("Odometry_Task", "Distance: %0.4fmm.", (get_l_distance() + get_r_distance()) / 2);

        set_last_l_pulses(l_pulses);
        set_last_r_pulses(r_pulses);
        stopped_recently = false;
    }

    // Record the total distance traveled when the robot stops moving.
    else if (stopped_recently == false) {
        ESP_LOGI("Odometry_Task", "Robot idle. Final Distance: %0.4fmm.", (get_l_distance() + get_r_distance()) / 2);

        l_encoder.reset_count();
        r_encoder.reset_count();
        set_last_l_pulses(0);
        set_last_r_pulses(0);
        set_l_distance(0.0);
        set_r_distance(0.0);
        stopped_recently = true;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return;
}
//  ============================================================


/*
    ============================================================
    Adjusts the speed for all 4 motors.
    ============================================================
*/
void Drive_Train::change_speed(uint32_t left_speed, uint32_t right_speed) {
    ESP_LOGI(config.name.c_str(), "Adjusting the train's speed.");

    config.Front_Driver.adjust_speed(left_speed, right_speed);
    config.Back_Driver.adjust_speed(left_speed, right_speed);
    return;
}
//  ============================================================


/*
    ============================================================
    Drives all motors forward.
    ============================================================
*/
void Drive_Train::move_forward() {
    ESP_LOGI(config.name.c_str(), "Making train move forward.");

    config.Front_Driver.forward();
    config.Back_Driver.forward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives all motors backward.
    ============================================================
*/
void Drive_Train::move_backward() {
    ESP_LOGI(config.name.c_str(), "Making train move backward.");

    config.Front_Driver.backward();
    config.Back_Driver.backward();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives the 2 left motors backward and 2 right motors forward
    to make robot turn left.
    ============================================================
*/
void Drive_Train::turn_left() {
    ESP_LOGI(config.name.c_str(), "Making train turn left.");

    config.Front_Driver.left();
    config.Back_Driver.left();
    return;
}
//  ============================================================


/*
    ============================================================
    Drives the 2 left motors forward and 2 right motors backward
    to make robot turn right.
    ============================================================
*/
void Drive_Train::turn_right() {
    ESP_LOGI(config.name.c_str(), "Making train turn right.");

    config.Front_Driver.right();
    config.Back_Driver.right();
    return;
}
//  ============================================================


/*
    ============================================================
    Stops all motors from spinning.
    ============================================================
*/
void Drive_Train::brake_all() {
    ESP_LOGI(config.name.c_str(), "Making train stop.");

    config.Front_Driver.brake();
    config.Back_Driver.brake();
    return;
}
//  ============================================================


/*
    ============================================================
    Returns the left wheel encoder object.
    ============================================================
*/
Wheel_Encoder& Drive_Train::get_left_encoder() {
    //ESP_LOGI(config.name.c_str(), "Retrieving left wheel encoder object.");
    return *(config.Left_Encoder);  // Returning a dereferenced pointer.
}
//  ============================================================


/*
    ============================================================
    Returns the right wheel encoder object.
    ============================================================
*/
Wheel_Encoder& Drive_Train::get_right_encoder() {
    //ESP_LOGI(config.name.c_str(), "Retrieving right wheel encoder object.");
    return *(config.Right_Encoder); // Returning a dereferenced pointer.
}
//  ============================================================


/*
    ============================================================
    Gets the wheel encoder's pulse count.
    ============================================================
*/
uint32_t Drive_Train::get_pulses(const Wheel_Encoder &encoder) {
    return encoder.get_pulse_count();
}
//  ============================================================


/*
    ============================================================
    Set the left wheel encoder's recorded distance.
    ============================================================
*/
void Drive_Train::set_l_distance(double distance) {
    l_distance = distance;
    return;
}
//  ============================================================


/*
    ============================================================
    Set the left wheel encoder's recorded distance.
    ============================================================
*/
double Drive_Train::get_l_distance() {
    return l_distance;
}
//  ============================================================


/*
    ============================================================
    Set the right wheel encoder's recorded distance.
    ============================================================
*/
void Drive_Train::set_r_distance(double distance) {
    r_distance = distance;
    return;
}
//  ============================================================


/*
    ============================================================
    Set the right wheel encoder's recorded distance.
    ============================================================
*/
double Drive_Train::get_r_distance() {
    return r_distance;
}
//  ============================================================


/*
    ============================================================
    Record the left wheel encoder's last pulse count.
    ============================================================
*/
void Drive_Train::set_last_l_pulses(uint32_t pulses) {
    last_l_pulses = pulses;
    return;
}
//  ============================================================

/*
    ============================================================
    Get the left wheel encoder's last pulse count.
    ============================================================
*/
uint32_t Drive_Train::get_last_l_pulses() {
    return last_l_pulses;
}
//  ============================================================


/*
    ============================================================
    Record the right wheel encoder's last pulse count.
    ============================================================
*/
void Drive_Train::set_last_r_pulses(uint32_t pulses) {
    last_r_pulses = pulses;
    return;
}
//  ============================================================

/*
    ============================================================
    Get the right wheel encoder's last pulse count.
    ============================================================
*/
uint32_t Drive_Train::get_last_r_pulses() {
    return last_r_pulses;
}
//  ============================================================