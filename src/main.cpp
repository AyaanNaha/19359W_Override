#include "main.h"

// Define drivetrain motors using your specified ports and PROS 4 syntax
// Right-side motors are inverted (-3 and -4) so pushing both sticks forward moves the robot straight.
pros::Motor left_front(-3, pros::v5::MotorGears::blue);
pros::Motor left_back(-4, pros::v5::MotorGears::blue);
pros::Motor right_front(1, pros::v5::MotorGears::blue); // Inverted
pros::Motor right_back(2, pros::v5::MotorGears::blue);  // Inverted

// Initialize the master controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    // Force high-speed motors to actively lock position on joystick release.
    left_front.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_back.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_front.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_back.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of VEX Competition Control.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to a competition switch.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. 
 */
void autonomous() {}

/**
 * Runs the user operator control code.
 */
void opcontrol() {
    // Deadzone threshold to ignore accidental joystick drift
    const int DEADZONE = 10;

    while (true) {
        // Tank Drive: Left stick controls left motors, Right stick controls right motors
        int left_joystick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_joystick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Apply deadzone filtering
        if (abs(left_joystick) < DEADZONE) left_joystick = 0;
        if (abs(right_joystick) < DEADZONE) right_joystick = 0;

        // Send power directly to your 600 RPM gearsets
        left_front.move(left_joystick);
        left_back.move(left_joystick);
        right_front.move(right_joystick);
        right_back.move(right_joystick);

        // Loop delay to prevent CPU overloading
        pros::delay(20);
    }
}
