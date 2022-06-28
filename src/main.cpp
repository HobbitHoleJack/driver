#include "main.h"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
#include <string>
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
		.withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 7_in, 1_in, 3.25_in}, quadEncoderTPR})
		.withOdometry() // use the same scales as the chassis (above)
		.buildOdometry(); // build an odometry chassis

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {


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
	pros::lcd::set_text(1, "Auton");
	pros::lcd::set_text(2, std::to_string(auton_side));


} 

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor top(1);
	pros::Motor bottom(2, true);

int L1_state{0};
bool L1_held{false};
top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

while(true) {
	// flywheel toggle
	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
		if (!L1_held){
			if (L1_state % 2 == 0){
				top.move(127);
				bottom.move(127);
				L1_state = L1_state + 1;
				L1_held = !L1_held;
			}
			else {
			top.brake();
			bottom.brake();
			L1_state = L1_state + 1;
			L1_held = !L1_held;
			}
		}
   }
	else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {L1_held = false;}
	//end flywheel toggle code

	pros::delay(20);
}
	}
