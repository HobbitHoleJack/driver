#include "main.h"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "gif-pros/gifclass.hpp"
using namespace okapi;
int auton_side = -1;

std::shared_ptr<OdomChassisController> chassis =
		ChassisControllerBuilder()
		.withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
		.withGains(
			{0.001, 0, 0.0001}, // distance controller gains
			{0.001, 0, 0.0001}, // turn controller gains
			{0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
		)
		.withSensors(
			ADIEncoder{'A', 'B'}, // left encoder in ADI ports A & B
			ADIEncoder{'C', 'D', true},  // right encoder in ADI ports C & D (reversed)
			ADIEncoder{'E', 'F'}  // middle encoder in ADI ports E & F
		)
		// green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
		// 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
		.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
		.withOdometry() // use the same scales as the chassis (above)
		.buildOdometry(); // build an odometry chassis


void on_center_button() {
	auton_side = 1;
	pros::lcd::set_text(1, "No auton");
}

void on_left_button() {
	auton_side = 0;
	pros::lcd::set_text(1, "Left side auton ready");
}

void on_right_button() {
	auton_side = 2;
	pros::lcd::set_text(1, "Right side auton ready");
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Please select an auton");

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
void competition_initialize() {
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);
}

void Odometry() {
	int l_enc, r_enc, b_enc;
	int old_l_enc{0}, old_r_enc{0}, old_b_enc{0};
	pros::ADIEncoder left_encoder ('A', 'B');
	pros::ADIEncoder right_encoder ('C', 'D');
	pros::ADIEncoder back_encoder ('E', 'F');
	l_enc = left_encoder.get_value();
	r_enc = right_encoder.get_value();
	b_enc = back_encoder.get_value();

}

void autonomous() {
	


	chassis->setState({0_in, 0_in, 0_deg});
} 

void opcontrol() {}
