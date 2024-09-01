#include "main.h"
#include "util.hpp"


using namespace util;

//Reset PID
void velPID::reset_variables() {
    output = 0;
    target = 0;
    error = 0;
    integral = 0;
    derivative = 0;
    prev_error = 0;
}

velPID::velPID() {
    reset_variables();
    set_constants(0, 0, 0, 0);
}

velPID::Constants velPID::return_constants() { return constants; }

//PID Constructor with constants
velPID::velPID(double p, double i, double d, double f) {
    reset_variables();
    set_constants(p, i, d, f);
}

//Set PID
void velPID::set_constants(double p, double i, double d, double f) {
    constants.kp = p;
    constants.ki = i;
    constants.kd = d;
    constants.kf = f;
}

void velPID::set_target(double input) { target = input; }
double velPID::return_target() { return target; }

//PID Math
double velPID::compute(double input) {
    error = target - input; //Proportion
    derivative = error - prev_error; //Derivative

    if (constants.ki != 0) {
        integral += error; //Integral
        
        if (sign(error) != sign(prev_error))
            integral = 0;
    }

    output = (error * constants.kp) + (integral * constants.ki) + (derivative * constants.kd) + (target * constants.kf);

    prev_error = error;

    return output;
}
