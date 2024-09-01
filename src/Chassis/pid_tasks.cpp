#include "chassis.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "util.hpp"

using util::clip_num;

using namespace util;

void Chassis::async_auto_task() {
  while (true) {

    switch (get_mode()) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN:
        turn_pid_task();
        break;
      case SWING:
        swing_pid_task();
        break;
      case ARC:
        arc_pid_task();
        break;
      case TO_POINT:
        move_to_point_task();
        break;
      case ANGLE_TO_POINT:
        angle_to_point_task();
        break;
      case DISABLE:
        break;  
    }

    if (pros::competition::is_autonomous() && !util::AUTON_RAN)
      util::AUTON_RAN = true;
  pros::delay(util::DELAY_TIME);
    }
}



void Chassis::move_to_point_task() {
    t += 10;
    double hypot = distance_to_point(target.x, target.y, current_direction); // straight line distance between target and current position
    pose projected = projected_point(5, target.x, target.y, 0, current_direction); // caculated projected point off of target
    double angleError = angle_to_point(target.x, target.y, current_direction); // use projected vector to calculate the smallest angle to point relative to the robot's heading

    double turnScale;
    if (turn_scale_on) 
      turnScale = 1 - fabs(angleError / InitialAngleError);
    else
      turnScale = 1;

    if (fabs(hypot) < deadband) {
      angleError = 0;
    }
    //Sets pid targets
    linearPID.set_target(hypot * get_tick_per_inch());
    angularPID.set_target(angleError);

    //set pid input to zero since we are changing pid targets every loop
    linearPID.compute(0);
    angularPID.compute(0);

    pros::lcd::clear_line(4);
    pros::lcd::print(4, "distance:%f angle:%f", hypot, angleError);


    
    //calculate the output of power
    double slewOut = slew_calculate(vertical_slew, vertical_position());


    
    //sums the drive and gyro output
  
    double lOut = (linearPID.output*turnScale) + (angularPID.output);   
    double rOut = (linearPID.output*turnScale) - (angularPID.output);
    
    double greatestSpeedRatio = std::max(fabs(lOut), fabs(rOut)) / slewOut;
    if (greatestSpeedRatio > 1.0) {
      lOut /= greatestSpeedRatio;
      rOut /= greatestSpeedRatio;
    }

    if (graph_var_toggle) 
      printf("%d,%.2f,%.2f,%.2f, %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", t, hypot, angleError, get_gyro(), linearPID.derivative, angularPID.derivative, linearPID.output, angularPID.output, lOut, rOut, current.x, current.y, projected.x, projected.y);
    if (drive_toggle)
      set_tank(lOut, rOut);
}


void Chassis::angle_to_point_task() {
    t+= 10;    
    //double angleError = angle_to_point(target.x, target.y, current_direction); // use projected vector to calculate the smallest angle to point relative to the robot's heading
    double xError = target.x - current.x;
    double yError = target.y - current.y;
    //is bot driving with the front or back?
    int flip = current_direction == REV ? 180 : 0;
    double angleError = util::wrap_angle_180((util::to_deg(atan2(xError, yError)) - flip) - get_gyro());
    //Sets pid targets
    turnPID.set_target(angleError);

    //set pid input to zero since we are changing pid targets every loop
    turnPID.compute(0);

    double gyro_out = util::clip_num(turnPID.output, max_speed, -max_speed);

    if (turnPID.constants.ki != 0 && (fabs(turnPID.return_target()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
        if (get_turn_min() != 0)
            gyro_out = util::clip_num(gyro_out, get_turn_min(), -get_turn_min());
    }
 if (graph_var_toggle) 
      printf("%d,%.2f,%.2f,%.2f, %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", t, angleError, get_gyro(), turnPID.derivative, turnPID.output, gyro_out, current.x, current.y, target.x, target.y);
    if (drive_toggle)
        set_tank(gyro_out, -gyro_out);
}


void Chassis::arc_pid_task() {
    arcPID.compute(get_gyro());
    double wheelRatio = fabs((circleRadius + CHASSIS_WIDTH/2) / (circleRadius - CHASSIS_WIDTH/2));
    double rawGyroOut = clip_num(arcPID.output, max_speed, -max_speed);

    if (circleRadius <= CHASSIS_WIDTH) {
        printf("Circle Radius cannot be less than or equal to Chassis Width\nCircle Radius must be greater than%f\n", CHASSIS_WIDTH);
        rawGyroOut = 0;
    }
    //printf("wheel ratio:%f\nRawGyro%f\n", wheelRatio, rawGyroOut);
    if (arcPID.constants.ki != 0 && (fabs(arcPID.return_target()) > arcPID.constants.start_i && fabs(arcPID.error) < arcPID.constants.start_i)) {
      if (get_turn_min() != 0)
        rawGyroOut = util::clip_num(rawGyroOut, get_turn_min(), -get_turn_min());
    }
  
    double rightArcOut;
    double leftArcOut;
    if(current_direction == LEFT) {
        rightArcOut = -rawGyroOut * wheelRatio;
        leftArcOut = -rawGyroOut * (1/wheelRatio);
    } else {
        rightArcOut = rawGyroOut * (1/wheelRatio);
        leftArcOut = rawGyroOut * wheelRatio;
    }

    if (drive_toggle)
      set_tank(leftArcOut, rightArcOut);
}




void Chassis::drive_pid_task() {
    t += 10;
    leftPID.compute(vertical_position());
    rightPID.compute(vertical_position());
    headingPID.compute(get_gyro());

    double l_slew_out = slew_calculate(left_slew, vertical_position());
    double r_slew_out = slew_calculate(right_slew, vertical_position());

    double l_drive_out = util::clip_num(leftPID.output, l_slew_out, -l_slew_out);
    double r_drive_out = util::clip_num(rightPID.output, r_slew_out, -r_slew_out);

    double gyro_out = heading_on ? headingPID.output : 0;

    double l_out = l_drive_out + gyro_out;
    double r_out = r_drive_out - gyro_out;

    if(graph_var_toggle)
        printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", t, rightPID.error, rightPID.derivative, r_drive_out, headingPID.error, headingPID.derivative, gyro_out);
    if (drive_toggle)
        set_tank(l_out, r_out);
}




void Chassis::turn_pid_task() {
    t += 10;
    turnPID.compute(get_gyro());
    double gyro_out = util::clip_num(turnPID.output, max_speed, -max_speed);

    if (turnPID.constants.ki != 0 && (fabs(turnPID.return_target()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
        if (get_turn_min() != 0)
            gyro_out = util::clip_num(gyro_out, get_turn_min(), -get_turn_min());
    }
    if(graph_var_toggle)
        printf("%d,%.2f,%.2f,%.2f,%.2f\n", t, turnPID.error, turnPID.integral, turnPID.derivative, gyro_out);
    if (drive_toggle)
        set_tank(gyro_out, -gyro_out);
}




void Chassis::swing_pid_task() {

    swingPID.compute(get_gyro());

    double swing_out = util::clip_num(swingPID.output, max_speed, -max_speed);

    if (swingPID.constants.ki != 0 && (fabs(swingPID.return_target()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
        if (get_swing_min() != 0)
            swing_out = util::clip_num(swing_out, get_swing_min(), -get_swing_min());
    }

    if (drive_toggle) {

        if (current_swing == LEFT_SWING)
            set_tank(swing_out, 0);
        else if (current_swing == RIGHT_SWING)
            set_tank(0, -swing_out);
    }
}