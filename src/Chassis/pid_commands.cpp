#include "chassis.hpp"
#include "main.h"
#include "util.hpp"

using namespace util;

void Chassis::set_max_speed(int speed) {
    max_speed = util::clip_num(abs(speed), 127, -127);
}

void Chassis::reset_pid_targets() {
    headingPID.set_target(0);
    leftPID.set_target(0);
    rightPID.set_target(0);
    linearPID.set_target(0);
    angularPID.set_target(0);
    forward_drivePID.set_target(0);
    backward_drivePID.set_target(0);
    turnPID.set_target(0);
    target.x = 0;
    target.y = 0;
    target.theta = 0;
}

void Chassis::set_angle(double angle) {
    headingPID.set_target(angle);
    reset_gyro(angle);
    current.angleDeg = angle;
}

void Chassis::set_mode(e_mode p_mode) {
    mode = p_mode;
}

void Chassis::set_turn_min(int min) { turn_min = abs(min); }
int Chassis::get_turn_min() { return turn_min; }

void Chassis::set_swing_min(int min) {  swing_min = abs(min); }
int Chassis::get_swing_min() { return swing_min; }

e_mode Chassis::get_mode() { return mode; }



void Chassis::move_to_point(e_dir direction, double x_target, double y_target, int speed, bool slew_on, double deadband, bool turn_scale_on) {
    TICKS_PER_INCH = get_tick_per_inch();

    if (print_toggle) printf("Drive Started... Target Cords: %f, %f", x_target, y_target);
    if (print_toggle) printf(" with Slew");
    if (print_toggle) printf("\n");

    set_max_speed(speed);
    bool is_backwards = false;
    c_start = vertical_position();
    target.x = x_target;
    target.y = y_target;
    current_direction = direction;
    this->deadband = deadband;
    this->turn_scale_on = turn_scale_on;
    double target_enc = 0;
    
    //sign of legs of triangle
    initxErrorSign = util::sign(target.x - current.x);
    inityErrorSign = util::sign(target.y - current.y);

    //figure out the hypot of the triangle and the angle the robot need to turn
    double hypot = distance_to_point(target.x, target.y, direction);
    pose projected = projected_point(5, target.x, target.y, 0, direction);
    double angleError = angle_to_point(projected.x, projected.y, direction);
    InitialAngleError = angleError;

    double turnScale;
    if (turn_scale_on) 
      turnScale = 1 - fabs(angleError / InitialAngleError);
    else
      turnScale = 1;

    target.theta = angleError;
    target_enc = hypot * get_tick_per_inch();
    
    //set pid targets
    linearPID.set_target(target_enc * turnScale);
    angularPID.set_target(angleError);

    slew_initialize(vertical_slew, slew_on, max_speed, target_enc, vertical_position(), c_start, is_backwards);

    if (graph_var_toggle) printf("t, linearError, AngularError, gyro, linearDerivative, angularDerivative, LinearPower, AngularPower, LeftPower, RightPower, X, Y, projectedX, projectedY\n");
    if (print_toggle) printf("Hypotenuse: %f\n", hypot);
    if (print_toggle) printf("Angle To Target: %f", angleError);
    set_mode(TO_POINT);
}

void Chassis::turn_to_point(e_dir direction, double x_target, double y_target, int speed) {
    if (print_toggle) printf("Drive Started... Target Cords: %f, %f", x_target, y_target);
    if (print_toggle) printf("\n");

    set_max_speed(speed);
    target.x = x_target;
    target.y = y_target;
    current_direction = direction;

    //double angleError = angle_to_point(target.x, target.y, direction);
    double xError = target.x - current.x;
    double yError = target.y - current.y;
    //is bot driving with the front or back?
    int flip = current_direction == REV ? 180 : 0;
    double angleError = util::wrap_angle_180((util::to_deg(atan2(xError, yError)) - flip) - get_gyro());

    turnPID.set_target(angleError);

    if (graph_var_toggle) printf("t, AngularError, gyro, angularDerivative, AngularPower, drivePower, X, Y, targetX, targetY\n");
    if (print_toggle) printf("Angle To Target: %f\n", angleError);
    set_mode(ANGLE_TO_POINT);
}


void Chassis::set_drive_pid(double target, int speed, bool slew_on, bool toggle_heading) {
    TICKS_PER_INCH = get_tick_per_inch();

    if (print_toggle) printf("Drive Started... Target Value: %f (%f ticks)", target, target * TICKS_PER_INCH);
    if (slew_on && print_toggle) printf( " with slew");
    if (print_toggle) printf("\n");
    if (graph_var_toggle) printf("t, error, derivative, power, gyroError, gyroDerivative, gyroPower\n");

    set_max_speed(speed);
    heading_on = toggle_heading;
    bool is_backwards = false;
    l_start = vertical_position();
    r_start = vertical_position();

    double l_target_encoder, r_target_encoder;
    
    l_target_encoder = l_start + (target * TICKS_PER_INCH);
    r_target_encoder = r_start + (target * TICKS_PER_INCH);

    if (l_target_encoder < l_start && r_target_encoder < r_start) {
        auto consts = backward_drivePID.return_constants();
        leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
        rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
        is_backwards = true;
    } else {
        auto consts = forward_drivePID.return_constants();
        leftPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
        rightPID.set_constants(consts.kp, consts.ki, consts.kd, consts.start_i);
        is_backwards = false;
    }

    leftPID.set_target(l_target_encoder);
    rightPID.set_target(r_target_encoder);

    slew_initialize(left_slew, slew_on, max_speed, l_target_encoder, vertical_position(), l_start, is_backwards);
    slew_initialize(right_slew, slew_on, max_speed, r_target_encoder, vertical_position(), r_start, is_backwards);

    set_mode(DRIVE);
}




void Chassis::set_arc_pid(e_dir direction, double target, double circle_radius, int speed) {
    if (print_toggle) printf("Arc Started... Target Value: %f\n", target);
    arcPID.set_target(target);
    headingPID.set_target(target);
    this->target.theta = target;

    set_max_speed(speed);
    circleRadius = circle_radius;
    current_direction = direction;

    set_mode(ARC);
}



void Chassis::set_turn_pid(double target, int speed) {

    if (print_toggle) printf("Turn Started... Target Value: %f\n", target);
    if (graph_var_toggle) printf("t, error, integral, derivative, power\n");

    turnPID.set_target(target);
    headingPID.set_target(target);
    this->target.theta = target;
    //this->pose.theta = target;
    set_max_speed(speed);

    set_mode(TURN);
}


void Chassis::set_swing_pid(e_swing type, double target, int speed) {

    if (print_toggle) printf("Swing Started... Target Value: %f\n", target);
    current_swing = type;

    swingPID.set_target(target);
    headingPID.set_target(target);
    this->target.theta = target;

    set_max_speed(speed);

    set_mode(SWING);
}