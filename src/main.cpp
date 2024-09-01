#include "main.h"
#include "chassis.hpp"
#include "Auton_Selector/auton_selector.hpp"
#include "autons.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "util.hpp"
#include <algorithm>
using namespace pros;
using namespace util;
Chassis chassis(
	{-18, -19, 20}, //-10

	{14, -12, 13},  //3


	1,

	3.25,

	480,

	1,

	pros::E_MOTOR_GEAR_600,

	12.25,

	0, 0.25, 2.5
);
Intake intake(
	{11},
	
	pros::E_MOTOR_GEAR_600
);

void initialize() {
 chassis.set_curve_default(2, 2);
 
/*
 chassis.toggle_auto_print(false);
 chassis.toggle_auto_graph_var(false);
 util::toggle_auton_in_driver(false);

 pros::delay(500);
 chassis.toggle_modify_curve_with_controller(false);
 chassis.set_curve_default(1.5, 1.5);
 constants();

 as::auton_selector.add_autons(	{
Auton("Test", test)
 });
 chassis.initialize();
 as::initialize();

*/
pros::lcd::initialize();
}


void disabled() {}


void competition_initialize() {
}


void autonomous() {
	move();
}


//pros::Motor diagnostic(1, pros::E_MOTOR_GEARSET_06, false);
void opcontrol() {
	bool toggle = false;
	bool intakeState = 0;
	pros::Motor intake1(2, true);
	pros::Motor intake2(17, true);
	Motor_Group intake({intake1,intake2});
	while (true) {
	  /*if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && toggle == false) {
		intakeState = !intakeState;
		toggle = true;
	  } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		toggle = false;
	  }
	  if (intakeState) {
		intake.move(127);
	  }
	  else {
		intake.move(0);
	  }*/
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			//intake.move(-127);
			intake.move(-127);
		} else {
			intake.move(0);
		}
	
		/*pros::lcd::clear_line(1);
		pros::lcd::print(1, "R Watt:%.2f L Watt:%.2f", intake.get_power(), intake.get_power());
		pros::lcd::clear_line(2);
		pros::lcd::print(2, "R Torque:%.2f L Torque:%.2f", intake.get_torque(), intake.get_torque());
		pros::lcd::clear_line(3);
		pros::lcd::print(3, "R Current:%.2f L Current:%.2f", intake.get_current_draw(), intake.get_current_draw());*/

		chassis.arcade(util::SPLIT);
	  /*
	  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && toggle == false) {
		driveState = !driveState;
		toggle = true;
	  } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		toggle = false;
	  }
	  if (driveState) {
	  	chassis.arcade_flipped(e_type::SPLIT);
		pros::lcd::clear_line(1);
		pros::lcd::set_text(1, "regular");
	  }
	  else {
	   	chassis.arcade_flipped_curvature(e_type::SPLIT);
		pros::lcd::clear_line(1);
		pros::lcd::set_text(1, "curvature");
	  }
	  */
		pros::delay(DELAY_TIME);
	}
}
