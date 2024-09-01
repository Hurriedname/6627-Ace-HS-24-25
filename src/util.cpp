#include "main.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "util.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace util {
    
    bool AUTON_RAN = true;
    bool DRIVING = true;
    bool AUTON_IN_DRIVER = false;

    void toggle_auton_in_driver(bool state) {
        AUTON_IN_DRIVER = state;
    }
    double to_deg(double rad) {
        return rad * (180 / M_PI);
    }

    double to_rad(double deg) {
        return deg * (M_PI / 180);
    }

    double wrap_angle_90(double theta) {
        while (theta > 90) theta -= 360;
        while (theta < -90) theta += 360;
        return theta;
    }
    double wrap_angle_180(double theta) {
        while (theta > 180) theta -= 360;
        while (theta < -180) theta += 360;
        return theta;
    }

    double wrap_angle_360(double theta) {
        while (theta > 360) theta -= 360;
        while (theta < 0) theta += 360;
        return theta;
    }

    double hypot(double a, double b) {
        return sqrt(pow(a, 2) + pow(b, 2));
    }
    
    double slope(double deltaX, double deltaY) {
        return (deltaY/deltaX);
    }

    int sign (double input) {
        if (input < 0)
            return -1;
        else if (input > 0)
            return 1;
        return 0;
    }

    bool is_reversed(double input) {
        if (input < 0) return true;
        return false;
    }

    double clip_num(double input, double max, double min) {
        if (input < min) {  return min;  }
        if (input > max) {  return max;  }
        return input;
    }
    

    
    double distance_to_point(double xTarget, double yTarget, e_dir direction) {
    double xError = xTarget - chassis.current.x;
    double yError = yTarget - chassis.current.y;
    //printf("x error: %f y error: %f\n", x_error, y_error);

    //is bot driving with the front or back?
    int sign = direction == REV ? -1 : 1;

    //flip the sign of the hypot if the bot overshoots the x and y
    if (util::sign(xError) != chassis.initxErrorSign && util::sign(yError) != chassis.inityErrorSign)
        sign = sign == 1 ? -1 : 1;

    double distance = util::hypot(xError, yError) * sign;
    return distance;
}

double angle_to_point(double xTarget, double yTarget, e_dir direction) {
    //x and y error
    double xError = xTarget - chassis.current.x;
    double yError = yTarget - chassis.current.y;
    
    //is bot driving with the front or back?
    int flip = direction == REV ? 180 : 0;

    //flip angle if the bot overshoots the x and y
    
    if (util::sign(xError) != chassis.initxErrorSign && util::sign(yError) != chassis.inityErrorSign)
        flip = flip == 180 ? 0 : 180;
    
    double error = util::wrap_angle_180((util::to_deg(atan2(xError, yError)) - flip) - chassis.get_gyro());
    return error;
}
    int delay;
    void print_with_delay(std::function<void ()> print, int interval) {
        delay += DELAY_TIME;
        if (delay >= interval) {
            print();
            delay = 0;
        }
    }


    std::string exit_to_string(exit_output input) {
        switch ((int)input) {
            case RUNNING:
        return "Running";
            case SMALL_EXIT:
        return "Small";
            case BIG_EXIT:
        return "Big";
            case VELOCITY_EXIT:
        return "Velocity";
            case mA_EXIT:
        return "mA";
            case ERROR_NO_CONSTANTS:
        return "Error: Exit condition constants not set!";
            default:
        return "Error: Out of bounds!";
        } 

    return "Error: Out of bounds!";
}


Test::Test(std::vector<int> test_motor_ports) {
    
    for (auto i : test_motor_ports) {
        pros::Motor temp(abs(i), is_reversed(i));
        test_motors.push_back(temp);
    }
}

void Test::benchmark() {
    
}

}