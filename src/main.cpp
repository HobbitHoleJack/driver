#include "main.h"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
#include <iterator>
#include <ostream>
#include <string>
#include <cmath>
using namespace okapi;
int auton_side = -1;

std::shared_ptr<OdomChassisController> chassis =
		ChassisControllerBuilder()
		.withMotors({-7, -8}, {9, 10}) // left motor is 1, right motor is 2 (reversed)
		.withGains(
			{0.001, 0, 0.0001}, // distance controller gains
			{.005, 0, 0.0003}, // turn controller gains
			{0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
		)
		.withSensors(
			ADIEncoder{'A', 'B', true}, // left encoder in ADI ports A & B
			ADIEncoder{'G', 'H'},  // right encoder in ADI ports C & D (reversed)
			ADIEncoder{'E', 'F', true}  // middle encoder in ADI ports E & F
		)
		// green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
		// 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
		.withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 5_in, 0_in, 3.25_in}, quadEncoderTPR})
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
	#define center 7
	#define right_to_center 3.5
	#define back_to_center 7
	double L, R, S;
	double Lr{0}, Rr{0};
	double local_offset;
	int l_enc{0}, r_enc{0}, b_enc{0};
	int old_l_enc{0}, old_r_enc{0}, old_b_enc{0};
	int h_at_reset{0};
	int abs_orientation{0};
	int angle_change;
	int old_abs_orientation;
	int avg_orientation;
	pros::ADIEncoder left_encoder ('A', 'B');
	pros::ADIEncoder right_encoder ('C', 'D');
	pros::ADIEncoder back_encoder ('E', 'F');
	while(1){

		l_enc = left_encoder.get_value();
		r_enc = right_encoder.get_value();
		b_enc = back_encoder.get_value();

		L = ((l_enc * (3.14159265359/180)) * 1.625) - old_l_enc;
		R =  ((r_enc * (3.14159265359/180)) * 1.625) - old_r_enc;
		S = ((b_enc * (3.14159265359/180)) * 1.625) - old_b_enc;

		old_l_enc = L;
		old_r_enc = R;
		old_b_enc = S;

		old_abs_orientation = abs_orientation;

		Lr = Lr + L;
		Rr = Rr + R;
		
		abs_orientation = h_at_reset + ((Lr - Rr) / center);

		angle_change = abs_orientation - old_abs_orientation;

		if (angle_change == 0){local_offset = S / R;}

		else{local_offset = (2 * (sin(angle_change/2))) * (((S / angle_change) + back_to_center) / ((R / angle_change) + right_to_center));}

		avg_orientation = old_abs_orientation + (angle_change/2);

		



		pros::delay(10);
	}



}

void autonomous() {
	pros::lcd::set_text(1, "Auton");
	pros::lcd::set_text(2, std::to_string(auton_side));


} 

void opcontrol() {
	#define L_drive_1 3
	#define L_drive_2 4
	#define R_drive_1 5
	#define R_drive_2 6
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor top(1);
	pros::Motor bottom(2, true);

	pros::Motor right_1(9);
	pros::Motor right_2(10);

	pros::Motor left_1(7, true);
	pros::Motor left_2(8,true);

int L1_state{0};
top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


while(true) {
	// flywheel toggle
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			if (L1_state % 2 == 0){
				top.move(127);
				bottom.move(127);
				L1_state = L1_state + 1;}
			else {
			top.brake();
			bottom.brake();
			L1_state = L1_state + 1;}}}

	//end flywheel toggle code

	//start drive code
	left_1.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
	left_2.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

	right_1.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
	right_2.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

	pros::delay(20);
}
