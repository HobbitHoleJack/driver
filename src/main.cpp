#include "main.h"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
#include <string>
int auton_side = -1;

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
	double L, R, S;
	double Lr{0}, Rr{0};
	int l_enc{0}, r_enc{0}, b_enc{0};
	int old_l_enc{0}, old_r_enc{0}, old_b_enc{0};
	int global_h{0};
	int abs_orientation{0};
	int angle_change;
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

		Lr = Lr + L;
		Rr = Rr + R;
		
		abs_orientation = global_h + ((Lr - Rr) / center);


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
				L1_held = !L1_held;}
			else {
			top.brake();
			bottom.brake();
			L1_state = L1_state + 1;
			L1_held = !L1_held;}}}

	else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {L1_held = false;}
	//end flywheel toggle code

	pros::delay(20);
}
	}
