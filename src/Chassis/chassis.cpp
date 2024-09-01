#include "display/lv_objx/lv_chart.h"
#include "main.h"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "util.hpp"

#include <list>

using namespace util;
pros::Rotation vertical_enc(8);
double vertical_conversion;
pros::Rotation horizontal_enc(15);
double horizontal_conversion;
// Chassis Constructor      (DELETED int imu_port because we don't use an imu)
Chassis::Chassis(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, pros::motor_gearset_e_t cart, double chassis_width, double left_offset, double right_offset, double center_offset)
    : imu(imu_port),
      async_auto([this] { this->async_auto_task(); }), //Asynchronous Tasks for Auton
      tracking([this] { this->tracking_task(); }) /* three wheel tracking task */ {

        for (auto i : left_motor_ports) { // setup for the left side of the chassis
            pros::Motor temp(abs(i),cart, is_reversed(i));
            left_motors.push_back(temp);
        }
        for (auto i : right_motor_ports) { // setup for the right side of the chassis
            pros::Motor temp(abs(i), cart, is_reversed(i));
            right_motors.push_back(temp);
        }

        //Math for the drive ticks
        WHEEL_DIAMETER = wheel_diameter;
        RATIO = ratio;
        CARTRIDGE = ticks;
        TICKS_PER_INCH = get_tick_per_inch();
        LEFT_OFFSET = left_offset;
        RIGHT_OFFSET = right_offset;
        CENTER_OFFSET = center_offset;

        set_defaults();
}



void Chassis::set_defaults() {
    imu.set_data_rate(5);
    //Sets default constants for the drive PID tasks
    headingPID = {11, 0, 20, 0, "Heading"};
    forward_drivePID = {0.45, 0, 5, 0, "Forward"};
    backward_drivePID = {0.45, 0, 5, 0, "Backward"};
    turnPID = {5, 0.003, 35, 15, "Turn"};
    swingPID = {7, 0, 45, 0, "Swing"};
    leftPID = {0.45, 0, 5, 0, "Left"};
    rightPID = {0.45, 0, 5, 0, "Right"};
    linearPID = {0.45, 0, 5, 0, "Linear"};
    angularPID = {5, 0.003, 35, 15, "Angular"};
    rightVelocityPID = {1, 0, 1, 1};
    leftVelocityPID = {1, 0, 1, 1};

    set_turn_min(30);
    set_swing_min(30);

    set_slew_min_power(80, 80);
    set_slew_distance(7, 7);

    // Exit conditions for the drive PID tasks
    set_exit_condition(turn_exit, 100, 3, 500, 7, 500, 500);
    set_exit_condition(swing_exit, 100, 3, 500, 7, 500, 500);
    set_exit_condition(drive_exit, 80, 50, 300, 150, 500, 500);

    toggle_modify_curve_with_controller(true);

    set_left_curve_buttons(pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_UP);
    set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_X);

    set_joystick_threshold(5);

    toggle_auto_drive(true);
    toggle_auto_print(true);

    }

//returns the ticks per inch of the chassis (in my case the ticks per in of the tracking wheels)
double Chassis::get_tick_per_inch() {
    CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

    TICKS_PER_REV = CARTRIDGE * RATIO;
    //TICKS_PER_REV = (50.0 * (3600.0 / CARTRIDGE)) * RATIO;  // with no cart, the encoder reads 50 counts per rotation

    TICKS_PER_INCH = (TICKS_PER_REV / CIRCUMFERENCE);
    return TICKS_PER_INCH;
}


void Chassis::set_pid_constants(PID *pid, double p, double i, double d, double _start_i) {
    pid->set_constants(p, i, d, _start_i);
}

//function to power the chassis
void Chassis::set_tank(int left, int right) {
    if (pros::millis() < 500) return;

    for (auto i : left_motors) {
        //if (!pto_check(i)) i.move_voltage(left * (12000.0 / 127.0));
        i.move_voltage(left * (12000.0 / 127.0));
    }
    for (auto i : right_motors) {
        //if (!pto_check(i)) i.move_voltage(right * (12000.0 / 127.0));
        i.move_voltage(right * (12000.0 / 127.0));
    }
}

void Chassis::set_tank_velocity(int left, int right) {
    if (pros::millis() < 500) return;
    leftVelocityPID.set_target(left);
    rightVelocityPID.set_target(right);
    for (auto i : left_motors) {
        //if (!pto_check(i)) i.move_voltage(left * (12000.0 / 127.0));
        i.move_voltage(leftVelocityPID.compute(left_velocity()));
    }
    for (auto i : right_motors) {
        //if (!pto_check(i)) i.move_voltage(right * (12000.0 / 127.0));
        i.move_voltage(rightVelocityPID.compute(right_velocity()));
    }
}

//sets the current limit of the drive motors
void Chassis::set_drive_current_limit(int mA) {
    if (abs(mA) > 2500) {
        mA = 2500;
    }
    CURRENT_MA = mA;
    for (auto i : left_motors) {
        //if (!pto_check(i)) i.set_current_limit(abs(mA));
        i.set_current_limit(abs(mA));
    }
    for (auto i : right_motors) {
        //if (!pto_check(i)) i.set_current_limit(abs(mA));
        i.set_current_limit(abs(mA));
    }
}

//resets drive motors and tracking wheels
void Chassis::reset_drive_sensors() {
    left_motors[0].tare_position();
    right_motors[0].tare_position();
    vertical_enc.reset();
    horizontal_enc.reset();
    /*
    l_enc.reset();
    r_enc.reset();
    */
    //c_enc.reset();
}

//Chassis Telemetry
int Chassis::right_position() { 
    return right_motors[0].get_position();
    //return center_position();
    }
int Chassis::right_velocity() { return right_motors.front().get_actual_velocity(); }
double Chassis::right_mA() { return right_motors.front().get_current_draw(); }
bool Chassis::right_over_current() { return right_motors.front().is_over_current(); }

int Chassis::left_position() { 
    return left_motors[0].get_position();
    //return center_position();
    }
int Chassis::left_velocity() { return left_motors.front().get_actual_velocity(); }
double Chassis::left_mA() { return left_motors.front().get_current_draw(); }
bool Chassis::left_over_current() {return left_motors.front().is_over_current(); }

//Fetch the postioning of the tracking Wheels in Degrees
double Chassis::vertical_position() {
    //Conversion of Centidegrees to Degrees
    vertical_conversion = vertical_enc.get_position()/100.00;
    return vertical_conversion;
    }
double Chassis::horizontal_position() { 
    //Conversion of Centidegrees to Degrees
    horizontal_conversion = horizontal_enc.get_position()/100.00; 
    return horizontal_conversion;
    }


//sets the chassis brake type
void Chassis::set_drive_brake(pros::motor_brake_mode_e_t brake_type) {
    CURRENT_BRAKE = brake_type;
    for (auto i : left_motors) {
        //if (!pto_check(i)) i.set_brake_mode(brake_type);
        i.set_brake_mode(brake_type);
    }
    for (auto i : right_motors) {
        //if (!pto_check(i)) i.set_brake_mode(brake_type);
        i.set_brake_mode(brake_type);
    }
}

//Gyro Telemetry
void Chassis::reset_gyro(double new_heading) { imu.set_rotation(new_heading); }
double Chassis::get_gyro() { return imu.get_rotation(); }

bool Chassis::imu_calibrate() {
    imu.reset();
    int time = 0;
    while (true) {
        time += util::DELAY_TIME;

        if (time >= 2000) {
            if (!(imu.get_status() & pros::c::E_IMU_STATUS_CALIBRATING)) {
                break;
            }
        }
        if (time >= 3000) {
            if (print_toggle) printf("no IMU plugged in, (took %d ms to realise that)\n", time);
            return false;
        }
        pros::delay(util::DELAY_TIME);
    }
    master.rumble(".");
    if (print_toggle) printf("IMU is done calibrating (took %d ms)\n", time);
    return true;
}

void Chassis::initialize() {
    imu_calibrate();
    reset_drive_sensors();
    reset_odom();
}

void Chassis::toggle_auto_drive(bool toggle) { drive_toggle = toggle; }
void Chassis::toggle_auto_print(bool toggle) { print_toggle = toggle; }
void Chassis::toggle_auto_graph_var(bool toggle) { graph_var_toggle = toggle; }