#include "main.h"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "autoSelect/selection.h"
#include <iostream>
#include <iterator>
#include <ostream>
#include <string>
#include <cmath>
using namespace pros;
int auton_side = -1;


int degrees;
double posX, posY;

void Odometry() {
	bool odom{true};
	#define center 5
	#define back_to_center 0
	int oldR{0}, oldL{0}, oldB{0}, head;
	double cst = M_PI * 3.25 / 360;
	double dX, dY, dH, dR, dL, dB;

	pros::ADIEncoder left ('A', 'B');
	pros::ADIEncoder right ('G', 'H', true);
	pros::ADIEncoder back ('E', 'F');
	while (odom) {
        dR = right.get_value() - oldR;
        dL = left.get_value() - oldL;
        dB = back.get_value() - oldB;
        
        dX = cst * (dB - back_to_center * ((dR + dL) / center));
        dY = cst * (dR + dL) / 2;
        dH = cst * (dR - dL) / center;
    
        head += dH;
        degrees = head * 180 / M_PI;
  
        posX += dX * cos(head) - dY * sin(head);
        posY += dX * sin(head) + dY * cos(head);

        oldR = right.get_value();
        oldL = left.get_value();
        oldB = back.get_value();

        pros::delay(10);
    }

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
Task obamatry(Odometry);

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
	pros::lcd::set_text(1, "Auton");
	pros::lcd::set_text(2, std::to_string(auton_side));


} 

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_1(7, true), left_2(8,true), right_2(10), right_1(9), bottom(2, true), top(1);;

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
	right_1.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

	left_2.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
	right_2.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

	pros::delay(20);
}
