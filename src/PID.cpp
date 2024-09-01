#include "main.h"
#include "util.hpp"


using namespace util;

//Reset PID
void PID::reset_variables() {
    output = 0;
    target = 0;
    error = 0;
    integral = 0;
    derivative = 0;
    prev_error = 0;
}

PID::PID() {
    reset_variables();
    set_constants(0, 0, 0, 0);
}

PID::Constants PID::return_constants() { return constants; }

//PID Constructor with constants
PID::PID(double p, double i, double d, double start_i, std::string name) {
    reset_variables();
    set_constants(p, i, d, start_i);
    set_name(name);
}

//Set PID
void PID::set_constants(double p, double i, double d, double _start_i) {
    constants.kp = p;
    constants.ki = i;
    constants.kd = d;
    constants.start_i = _start_i;
}

//Set PID Exit Condition Timeouts
void PID::set_exit_condition(double _small_error, int _small_exit_time, double _big_error, int _big_exit_time, int _velocity_exit_time, int _mA_timeout) {
    exit.small_error = _small_error;
    exit.small_exit_time = _small_exit_time;
    exit.big_error = _big_error;
    exit.big_exit_time = _big_exit_time;
    exit.velocity_exit_time = _velocity_exit_time;
    exit.mA_timeout = _mA_timeout;
}


void PID::set_target(double input) { target = input; }
double PID::return_target() { return target; }

//PID Math
double PID::compute(double input) {
    error = target - input; //Proportion
    derivative = error - prev_error; //Derivative

    if (constants.ki != 0) {
        if (fabs(error) < constants.start_i)
            integral += error; //Integral
        
        if (sign(error) != sign(prev_error))
            integral = 0;
    }

    output = (error * constants.kp) + (integral * constants.ki) + (derivative * constants.kd);

    prev_error = error;

    return output;
}

//Reset Exit Condition Timers
void PID::reset_timers() {
    big_timer = 0;
    small_timer = 0;
    vel_timer = 0;
    current_timer = 0;
    is_mA = false;
}


void PID::set_name(std::string _name) {
    name = _name;
    is_name = name == "" ? false : true;
}

//Print Exit Condition Type
void PID::print_exit(exit_output exit_type) {
    std::cout << " ";
    if (is_name) 
        std::cout << name << " PID " << exit_to_string(exit_type) << "Exit.\n";
    else
        std::cout << exit_to_string(exit_type) << "Exit.\n";    
}

//Exit Condition Logic
exit_output PID::exit_condition(bool print) {
    //Prints Error if all the exit conditions are set to 0
    if (!(exit.small_error && exit.small_exit_time && exit.big_error && exit.big_exit_time && exit.velocity_exit_time && exit.mA_timeout)) {
        print_exit(ERROR_NO_CONSTANTS);
        return ERROR_NO_CONSTANTS;
    }

    //Once the robot gets within a set threshold of the target, make sure it is within that threshold for small_timer amount of time
    if (exit.small_error != 0) {
        if (std::abs(error) < exit.small_error) {
            small_timer += DELAY_TIME;
            big_timer = 0; //While running, big thresh does not run
            if (small_timer > exit.small_exit_time) {
                reset_timers();
                if (print) print_exit(SMALL_EXIT);
                return SMALL_EXIT;
            }
        } else {
            small_timer = 0;
        }
    }

    //Once the robot gets near the target, set a timer. 
    //If the robot does not get closer for big_timer amout of time, exit and move on
    if (exit.big_error != 0 && exit.big_exit_time != 0) {
        if (std::abs(error) < exit.big_error) {
            big_timer += DELAY_TIME;
            if (big_timer > exit.big_exit_time) {
                reset_timers();
                if (print) print_exit(BIG_EXIT);
                return BIG_EXIT;
            }
        } else {
            big_timer = 0;
        }
    }

    //If the motor velocity is zero(uses the derivative to check the velocity) code will timeout and interfered will be set to true
    if (exit.velocity_exit_time != 0) {
        if (std::abs(derivative) <= 0.05) {
            vel_timer += DELAY_TIME;
            if (vel_timer > exit.velocity_exit_time) {
                reset_timers();
                if (print) print_exit(VELOCITY_EXIT);
                return VELOCITY_EXIT;
            }
        } else {
            vel_timer = 0;
        }
    }

    return RUNNING;
}

exit_output PID::exit_condition(pros::Motor sensor, bool print) {
    //If there is too much resistance in the motor(current is too high), code will timeout and interfered will be set to true
    if (exit.mA_timeout != 0) {
        if (sensor.is_over_current()) {
            current_timer += DELAY_TIME;
            if (current_timer > exit.mA_timeout) {
                reset_timers();
                if (print) print_exit(mA_EXIT);
                return mA_EXIT;
            }
        } else {
            current_timer = 0;
        }
    }
    return exit_condition(print);
}

exit_output PID::exit_condition(std::vector<pros::Motor> sensor, bool print) {
    //If there is too much resistance in the motors(current is too high), code will timeout and interfered will be set to true
    if (exit.mA_timeout != 0) {
        for (auto i : sensor) {
            if(i.is_over_current()) {
                is_mA = true;
                break;
            }

            else {//If not all of the motors are pulling too much current, keep is_mA false
                is_mA = false;
            }
        }
        if (is_mA) {
            current_timer += DELAY_TIME;
            if (current_timer > exit.mA_timeout) {
                reset_timers();
                if (print) print_exit(mA_EXIT);
                return mA_EXIT;
            }
        } else {
            current_timer = 0;
        }
    }

    return exit_condition(print);
}   


