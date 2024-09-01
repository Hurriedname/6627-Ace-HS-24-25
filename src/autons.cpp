#include "autons.hpp"
#include "VelocityPID.hpp"
#include "chassis.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "util.hpp"
#include <functional>


const int DRIVE_SPEED = 115;
const int TURN_SPEED = 90;
const int SLOW_SPEED = 60;
const int POINT_SPEED = 90;

void constants() {
    chassis.set_exit_condition(chassis.turn_exit, 3, 80, 7, 300, 500, 500);
    chassis.set_exit_condition(chassis.arc_exit, 3, 100, 7, 300, 500, 500);
    chassis.set_exit_condition(chassis.swing_exit, 3, 100, 7, 500, 500, 500);
    chassis.set_exit_condition(chassis.drive_exit, 50, 60, 150, 300, 500, 500);
    chassis.set_exit_condition(chassis.linear_exit, 50, 70, 150, 300, 50000000, 1000);
    chassis.set_exit_condition(chassis.angular_exit, 75, 50, 150, 300, 5000000, 500);

    chassis.set_slew_min_power(50, 50);
    chassis.set_slew_distance(8, 8);
    chassis.set_drive_current_limit(5000);

    chassis.set_pid_constants(&chassis.headingPID, 11, 0, 15, 0);
    //chassis.set_pid_constants(&chassis.forward_drivePID, 0.475, 0, 5, 0);
    //chassis.set_pid_constants(&chassis.backward_drivePID, 0.385, 0, 3.55, 0);
    //chassis.set_pid_constants(&chassis.forward_drivePID, 0.385, 0, 3.55, 0);
    chassis.set_pid_constants(&chassis.backward_drivePID, 0.235, 0, 2, 0);
    chassis.set_pid_constants(&chassis.forward_drivePID, 0.235, 0, 2, 0);
    chassis.set_pid_constants(&chassis.turnPID, 5, 0.0035, 44.25, 15);
    chassis.set_pid_constants(&chassis.linearPID, 0.235, 0, 2, 0);
    chassis.set_pid_constants(&chassis.angularPID, 5, 0.00, 44.5, 15);
    chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45.5, 0);
    chassis.set_pid_constants(&chassis.arcPID, 3.675, 0, 44.25, 0);
}

pros::Motor diagnostic(1, pros::E_MOTOR_GEARSET_06, false);

void motorTest() {
  diagnostic.move_voltage(12000);
	double accumulatedVelocity = 0;
	int count = 0;
	while (true) {
	  if (count>= 3000) break;
	  diagnostic.move_voltage(12000);
	  accumulatedVelocity += diagnostic.get_actual_velocity() * 0.75;
	  pros::lcd::clear_line(1);
	  pros::lcd::clear_line(2);
	  pros::lcd::print(1, "Current Velocity: %.0f", (diagnostic.get_actual_velocity()*(.75)) );
	  pros::lcd::print(2, "Current Wattage: %.0f", (diagnostic.get_power()));

	  count++;
		pros::delay(DELAY_TIME);
	}
	diagnostic.move_velocity(0);
	pros::lcd::print(2, "Averaged Velocity: %.0f", (accumulatedVelocity/count));
}

//pros mut -af run > velocitypidData.csv
void test() {
  chassis.rightVelocityPID.set_constants(40, 0, 10, 17);
  chassis.leftVelocityPID.set_constants(40, 0, 10, 17);
  int time = 0;
  printf("time, vel, power, proportion, target\n");
  while (1) {
  chassis.set_tank_velocity(500, 500);
	  pros::lcd::clear_line(1);
	  pros::lcd::clear_line(2);
	  pros::lcd::print(1, "rVel: %d lVel: %d", chassis.right_velocity(), chassis.left_velocity());
	  pros::lcd::print(2, "rVolt: %.2f lVolt: %.2f", chassis.rightVelocityPID.output, chassis.leftVelocityPID.output);
    printf("%d, %d, %f, %f, %f\n", time, chassis.left_velocity(), chassis.leftVelocityPID.output, (chassis.leftVelocityPID.error*40), chassis.leftVelocityPID.target);
    time += 10;
    pros::delay(10);
  }
}

void move(){
  chassis.move_to_point(FWD, 0, 12, 100);
}