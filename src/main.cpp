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

//global odom variables 

int degrees;
double posX, posY;
bool odom{false};

void Odometry() {
	#define center 5 // distance between side wheels
	#define back_to_center 0 // yes
	int oldR{0}, oldL{0}, oldB{0}, head, R, L, B;
	double cst = M_PI * 3.25 / 360; // inches per encoder tick
	double dX, dY, dH, dR, dL, dB;

	pros::ADIEncoder left ('A', 'B', true);
	pros::ADIEncoder right ('G', 'H');
	pros::ADIEncoder back ('E', 'F', true);
	while (odom) {

		R = right.get_value();
		L = left.get_value();
		B = back.get_value();

        dR = R - oldR;
        dL = L - oldL;  // getting the change in degrees from the last cycle
        dB = B - oldB;
        
        dX = cst * (dB - back_to_center * ((dR + dL) / center));
        dY = cst * (dR + dL) / 2;									// calculate change in x, y, and heading
        dH = cst * (dR - dL) / center;
    
        head += dH;
        degrees = head * 180 / M_PI;	// add the change in heading to the global heading, and then convert to degrees
  
        posX += dX * cos(head) - dY * sin(head);
        posY += dX * sin(head) + dY * cos(head); // add the change in x and y to the global position

        oldR = R;
        oldL = L; // update old encoder values
        oldB = B;

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

while(true) {
	std::string heading = std::to_string(degrees);
	std::string x = std::to_string(posX);
	std::string y = std::to_string(posY);
	lcd::print(1, heading.c_str());
	lcd::print(2, x.c_str());
	lcd::print(3, y.c_str());
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
