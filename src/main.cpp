#include "main.h"

// --- Device Declarations ---

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Drivetrain Motors (Blue - 600 RPM | Right side reversed using negative ports)
pros::Motor left_front(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_back(19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_front(-11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_back(-12, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// PROS 4: Motor Groups make tank drive much easier to code
pros::MotorGroup left_drive({20, 19});
pros::MotorGroup right_drive({-11, -12});

// Lift Motors (Green - 200 RPM | Port 10 is reversed using negative port)
pros::Motor lift_1(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor lift_10(-10, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::MotorGroup lift({1, -10}); // Grouping the lift makes dual-motor control cleaner

// Claw Rotation Motor (Red - 100 RPM)
pros::Motor claw_rot(-2, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);

// Claw Pneumatic Piston (Updated PROS 4 namespace: pros::adi)
pros::adi::DigitalOut claw_piston('H');

// --- Initialization ---
void initialize() {
    // MotorGroups allow you to tare all motors in the group at once
    lift.tare_position(); 
    claw_rot.tare_position();

    // MotorGroups also allow you to set brake modes together
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw_rot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}
void competition_initialize() {}

// --- Autonomous Routine ---
void autonomous() {
    // Write your autonomous routine here. 
    /* Example: Move forward for 1 second using Motor Groups
    left_drive.move(127);
    right_drive.move(127);
    pros::delay(1000);
    
    left_drive.move(0);
    right_drive.move(0);
    */
}

// --- Driver Control ---
void opcontrol() {
    bool piston_state = false;
    bool piston_button_pressed = false;

    bool claw_rot_state = false; // false = 0 deg, true = 90 deg
    bool claw_rot_button_pressed = false;

    while (true) {
        // 1. Tank Drive Control (Simplified with Motor Groups)
        int left_power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        left_drive.move(left_power);
        right_drive.move(right_power);

        // 2. Lift Control (L1 Up, L2 Down) with Lower Hard Limit
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(50); // Moves both lift_1 and lift_10
        } 
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (lift_1.get_position() > 0) {
                lift.move(-50);
            } else {
                lift.move(0);
            }
        } 
        else {
            lift.move(0);
        }

        // 3. Claw Piston Actuation (Toggle on R2)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (!piston_button_pressed) {
                piston_state = !piston_state; 
                claw_piston.set_value(piston_state);
                piston_button_pressed = true; 
            }
        } else {
            piston_button_pressed = false; 
        }

        // 4. Claw Rotation 90 Degrees (Toggle on R1)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (!claw_rot_button_pressed) {
                claw_rot_state = !claw_rot_state; 
                claw_rot_button_pressed = true;

                if (claw_rot_state) {
                    claw_rot.move_absolute(90, 100); 
                } else {
                    claw_rot.move_absolute(0, 100);  
                }
            }
        } else {
            claw_rot_button_pressed = false;
        }

        pros::delay(20);
    }
}