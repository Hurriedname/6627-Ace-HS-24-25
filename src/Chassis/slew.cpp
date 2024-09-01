#include "main.h"



void Chassis::set_slew_min_power(int fwd, int rev) {
    SLEW_MIN_POWER[0] = abs(fwd);
    SLEW_MIN_POWER[1] = abs(rev);
}

void Chassis::set_slew_distance(int fwd, int rev) {
    SLEW_DISTANCE[0] = abs(fwd);
    SLEW_DISTANCE[1] = abs(rev);
}

void Chassis::slew_initialize(slew_ &input, bool slew_on, double max_speed, double target, double current, double start, bool backwards) {
    input.enabled = slew_on;
    input.max_speed = max_speed;

    input.sign = util::sign(target - current);
    input.x_intercept = start + ((SLEW_DISTANCE[backwards] * input.sign) * TICKS_PER_INCH);
    input.y_intercept = max_speed * input.sign;
    input.slope = ((input.sign * SLEW_MIN_POWER[backwards]) - input.y_intercept) / (input.x_intercept - 0 - start);
}

double Chassis::slew_calculate(slew_ &input, double current) {

    if (input.enabled) {
        input.error = input.x_intercept - current;

        if (util::sign(input.error) != input.sign)
            input.enabled = false;
        
        else if (util::sign(input.error) == input.sign)
            return ((input.slope * input.error) + input.y_intercept) * input.sign;
    }

    return max_speed;
}

