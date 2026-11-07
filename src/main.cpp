#include "main.h"
#include <cmath>

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
// --- Autonomous Constants ---
const double WHEEL_DIAMETER = 2.75;
const double GEAR_RATIO = 60.0 / 36.0; // Driven / Driving
const double DEGREES_PER_INCH = (360.0 * GEAR_RATIO) / (WHEEL_DIAMETER * M_PI);

// --- Autonomous Helper Functions ---

/**
 * Moves the robot forward or backward a specific distance.
 * @param inches Distance to move (negative for backward).
 * @param speed Motor speed (0 to 127).
 */
void drive_inches(double inches, int speed) {
    // Calculate target degrees
    double target_degrees = inches * DEGREES_PER_INCH;
    
    // Ensure speed is correctly signed based on direction
    if (inches < 0 && speed > 0) speed = -speed;
    else if (inches > 0 && speed < 0) speed = -speed;

    // Reset encoders to 0 before moving
    left_front.tare_position();
    left_back.tare_position();
    right_front.tare_position();
    right_back.tare_position();

    // Command motors to move to the relative target
    left_front.move_relative(target_degrees, speed);
    left_back.move_relative(target_degrees, speed);
    right_front.move_relative(target_degrees, speed);
    right_back.move_relative(target_degrees, speed);

    // Wait until the motors reach their target (within a 5-degree threshold)
    while (std::abs(left_front.get_position() - target_degrees) > 5) {
        pros::delay(20);
    }
}

/**
 * Turns the robot in place.
 * Note: You will need to tune the 'degrees_per_turn_degree' multiplier 
 * based on your robot's track width and friction.
 * @param degrees Degrees to turn (positive = right, negative = left).
 * @param speed Motor speed (0 to 127).
 */
void turn(double degrees, int speed) {
    // Tuning constant: how many motor degrees equal 1 degree of robot rotation.
    // Start with 3.0 and adjust based on testing.
    double turn_multiplier = 3.0; 
    double target_degrees = degrees * turn_multiplier;

    left_front.tare_position();
    left_back.tare_position();
    right_front.tare_position();
    right_back.tare_position();

    // Right turns: Left moves forward, Right moves backward
    left_front.move_relative(target_degrees, speed);
    left_back.move_relative(target_degrees, speed);
    right_front.move_relative(-target_degrees, speed);
    right_back.move_relative(-target_degrees, speed);

    while (std::abs(left_front.get_position() - target_degrees) > 5) {
        pros::delay(20);
    }
}

// --- Autonomous Routine ---
void autonomous() {
    // Example routine using the new functions:
    
    drive_inches(24, 100);  // Drive forward 24 inches at speed 100
    pros::delay(200);       // Brief pause to let the robot settle
    
    turn(90, 80);           // Turn right 90 degrees
    pros::delay(200);
    
    drive_inches(-12, 100); // Back up 12 inches
}

// --- Driver Control ---
void opcontrol() {
    bool piston_state = false;
    bool piston_button_pressed = false;

    bool claw_rot_state = false; // false = 0 deg, true = 90 (outward)
    bool claw_rot_button_pressed = false;
    bool is_manual_claw = false; 

    while (true) {
        // 1. Tank Drive Control
        int left_power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        left_drive.move(left_power);
        right_drive.move(right_power);

        // --- NEW: Read Lift Position for Interlocks ---
        double lift_pos = lift_1.get_position();
        bool in_danger_zone = (lift_pos < 0);

        // 2. Lift Control (L1 Up, L2 Down)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(127); 
        } 
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // New limit: 3 rotations in reverse (3 * 360 = 1080 degrees)
            if (lift_pos > -1080) {
                lift.move(-127);
            } else {
                lift.move(0);
            }
        } 
        else {
            lift.move(0);
        }

        // --- NEW: Automatic Safety Interlock ---
        // If the lift drops below 0 and the claw isn't outward, force it out
        if (in_danger_zone && !claw_rot_state) {
            claw_rot_state = true;
            claw_rot.move_absolute(90, 100);
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

        // 4. Claw Rotation (Manual on UP/DOWN, Toggle on R1)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            claw_rot.move(50); // Move outward
            is_manual_claw = true;
        } 
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            // Interlock: Prevent manual retracting in the danger zone
            if (!in_danger_zone) {
                claw_rot.move(-50); // Safe to move inward
            } else {
                claw_rot.move(0);   // Blocked
            }
            is_manual_claw = true;
        } 
        else {
            if (is_manual_claw) {
                claw_rot.move(0); 
                is_manual_claw = false;
            }

            // Toggle on R1
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                if (!claw_rot_button_pressed) {
                    
                    // Interlock: Block R1 retracting if lift is in danger zone
                    if (in_danger_zone && claw_rot_state == true) {
                        master.rumble("-"); // Vibrate to tell driver they are locked out!
                    } 
                    else {
                        // Safe to toggle normally
                        claw_rot_state = !claw_rot_state; 
                        if (claw_rot_state) {
                            claw_rot.move_absolute(90, 100); 
                        } else {
                            claw_rot.move_absolute(0, 100);  
                        }
                    }
                    claw_rot_button_pressed = true;
                }
            } else {
                claw_rot_button_pressed = false;
            }
        }

        pros::delay(20);
    }
}