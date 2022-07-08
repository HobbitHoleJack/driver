#include "main.h"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include <iterator>
#include <ostream>
#include <string>
#include <cmath>
using namespace pros;
int auton_side = -1;

//global odom variables 

double degrees{0};
double posX{0}, posY{0};
bool odom{false};

void Odometry() {
	#define center 6.5 // distance between side wheels
	#define back_to_center 3.75 // yes
	int oldR{0}, oldL{0}, oldB{0};
	double In_per_tick = M_PI * 3.25 / 360; // inches per encoder tick
	double dX{0}, dY{0}, dH{0}, dR{0}, dL{0}, dB{0}, head{0}, R{0}, L{0}, B{0};

	pros::ADIEncoder left ('A', 'B', true);
	pros::ADIEncoder right ('G', 'H');
	pros::ADIEncoder back ('E', 'F', true);
  left.reset();
  right.reset();
  back.reset();
	while (true) {
        oldR = R;
        oldL = L; // update old encoder values
        oldB = B;

        R = right.get_value();
        L = left.get_value();
        B = back.get_value();

        dR = R - oldR;
        dL = L - oldL;  // getting the change in degrees from the last cycle
        dB = B - oldB;
        
				// calculate change in x, y, and heading
        dH = In_per_tick * (dR - dL) / center;
        dX = In_per_tick * (dL + dR) / 2.0;
        dY = In_per_tick * (dB - (dR - dL) * back_to_center / center);
    
        //head += dH;
        //degrees = head * 180 / M_PI;	// add the change in heading to the global heading, and then convert to degrees
  
        head = degrees + (dH / 2);
        posX += dX * cos(head) - dY * sin(head);
        posY += dX * sin(head) + dY * cos(head); // add the change in x and y to the global position
        degrees += dH;
		lcd::print(1, std::to_string(degrees).c_str());
    lcd::print(2, std::to_string(posX).c_str());
    lcd::print(3, std::to_string(posY).c_str());
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
	pros::lcd::initialize();
	Task odom(Odometry);
	pros::lcd::print(0, "odom started");
	pros::delay(10);
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

#define LEFT_WHEELS_PORT_1 7
#define RIGHT_WHEELS_PORT_1 19
#define LEFT_WHEELS_PORT_2 8
#define RIGHT_WHEELS_PORT_2 20

void opcontrol() {
  bool fly_state = false;
  pros::Motor left_wheels (LEFT_WHEELS_PORT_1, true);
  pros::Motor right_wheels (RIGHT_WHEELS_PORT_1);
  pros::Motor left_wheels_2 (LEFT_WHEELS_PORT_2, true);
  pros::Motor right_wheels_2 (RIGHT_WHEELS_PORT_2);
  pros::Controller master (CONTROLLER_MASTER);
  pros::Motor top (10);
  pros::Motor bottom (9, true);

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
        L1_state = L1_state + 1;}}
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    int left = power + turn;
    int right = power - turn;
    left_wheels.move(left);
    right_wheels.move(right);
    left_wheels_2.move(left);
    right_wheels_2.move(right);

    pros::delay(20);
  }
}
