#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "h");
}

void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void autonomous() {
	#define LR_distance_from_center 5
	#define pi 3.14159
	double x_pos = 0;
	double y_pos = 0;
	int heading_rad = 0;
	int heading;

	pros::ADIEncoder left_enc ('A', 'B', false);
	pros::ADIEncoder right_enc ('C', 'D', false);
	pros::ADIEncoder back_enc ('E', 'F', false);
	// 1 tick on encoders should == 0.0283616"

	heading_rad = (left_enc.get_value() - right_enc.get_value()) / LR_distance_from_center;
	heading = heading_rad * (180/pi);
	
} 

void opcontrol() {}
