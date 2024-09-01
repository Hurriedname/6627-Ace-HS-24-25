#include "chassis.hpp"
#include "pros/motors.hpp"
#include "util.hpp"
#include "main.h"

using namespace util;

void Chassis::set_exit_condition(int type, double _small_error, int _small_exit_time, double _big_error, int _big_exit_time, int _velocity_exit_time, int _mA_timeout) {
    //sets exit conditions of all drive pids
    if (type == drive_exit) {
        leftPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
        rightPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
    }
    if (type == linear_exit) {
        linearPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
    }
    if (type == angular_exit) {
        angularPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
    }

    if (type == turn_exit) {
        turnPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
    }
    if (type == arc_exit) {
        arcPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
    }

    if (type == swing_exit) {
        swingPID.set_exit_condition(_small_error, _small_exit_time, _big_error, _big_exit_time, _velocity_exit_time, _mA_timeout);
    }
}

void Chassis::wait_drive() {
    //function that locks the program until exit conditions of a PID function are met
    pros::delay(util::DELAY_TIME);

    if (get_mode() == DRIVE) {
        exit_output left_exit = RUNNING;
        exit_output right_exit = RUNNING;
        while(left_exit == RUNNING || right_exit == RUNNING) {
            left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
            right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
            pros::delay(util::DELAY_TIME);
        }
        if (print_toggle) std::cout << " Left: " << exit_to_string(left_exit) << " Exit.    Right: " << exit_to_string(right_exit) << " Exit.\n";

        if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
    else if (get_mode() == TO_POINT) {
        exit_output linear_exit = RUNNING;
        //exit_output turn_exit = RUNNING;
        while (linear_exit == RUNNING) {
            linear_exit = linear_exit != RUNNING ? linear_exit : linearPID.exit_condition(left_motors[0]);
            //turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
            pros::delay(DELAY_TIME);
        }
        if (print_toggle) std::cout << " To Point: " << exit_to_string(linear_exit) << "Exit.\n";

        if (linear_exit == mA_EXIT || linear_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
    else if (get_mode() == TURN) {
        exit_output turn_exit = RUNNING;
        while (turn_exit == RUNNING) {
            turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
            pros::delay(DELAY_TIME);
        }
        if (print_toggle)  {
            std::cout << " Turn: " << exit_to_string(turn_exit) << " Exit.\n";
            printf("Actual: %f  Target: %f\n", get_gyro(), turnPID.return_target());
        }

        if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
    else if (get_mode() == ANGLE_TO_POINT) {
        exit_output angular_exit = RUNNING;
        while (angular_exit == RUNNING) {
            angular_exit = angular_exit != RUNNING ? angular_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
            pros::delay(DELAY_TIME);
        }
        headingPID.set_target(get_gyro());
        if (print_toggle)  {
            std::cout << " Angle: " << exit_to_string(angular_exit) << " Exit.\n";
            printf("Actual: %f  Target: %f\n", get_gyro(), turnPID.return_target());
        }

        if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
    else if (get_mode() == ARC) {
        exit_output turn_exit = RUNNING;
        while (turn_exit == RUNNING) {
            turn_exit = turn_exit != RUNNING ? turn_exit : arcPID.exit_condition({left_motors[0], right_motors[0]});
            pros::delay(DELAY_TIME);
        }
        if (print_toggle) std::cout << " Arc: " << exit_to_string(turn_exit) << " Exit.\n";

        if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
    else if (get_mode() == SWING) {
        exit_output swing_exit = RUNNING;
        pros::Motor& sensor = current_swing == util::LEFT_SWING ? left_motors[0] : right_motors[0];
        while (swing_exit == RUNNING) {
            swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
            pros::delay(util::DELAY_TIME);
        }
        if (print_toggle) std::cout << " SWING: " << exit_to_string(swing_exit) << " Exit.\n";

        if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
            interfered = true;
        }
    }
}

void Chassis::wait_until(double target) {
    //function to lock the code until the pid function has met a cetain target
    if (get_mode() == DRIVE) {
        double l_target = l_start + (target * TICKS_PER_INCH);
        double r_target = r_start + (target * TICKS_PER_INCH);
        double l_error = l_target - vertical_position();
        double r_error = r_target - vertical_position();
        int l_sgn = util::sign(l_error);
        int r_sgn = util::sign(r_error);

        exit_output left_exit = RUNNING;
        exit_output right_exit = RUNNING;

        while (true) {
            l_error = l_target - vertical_position();
            r_error = r_target - vertical_position();

            if (util::sign(l_error) == l_sgn || util::sign(r_error) == r_sgn) {
                if (left_exit == RUNNING || right_exit == RUNNING) {
                    left_exit = left_exit != RUNNING ? left_exit : leftPID.exit_condition(left_motors[0]);
                    right_exit = right_exit != RUNNING ? right_exit : rightPID.exit_condition(right_motors[0]);
                    pros::delay(util::DELAY_TIME);
                } else {
                    if (print_toggle) std::cout << " Left: " << exit_to_string(left_exit) << " Wait Until Exit. Right: " << exit_to_string(right_exit) << " Wait Until Exit.\n";
                    
                    if (left_exit == mA_EXIT || left_exit == VELOCITY_EXIT || right_exit == mA_EXIT || right_exit == VELOCITY_EXIT) {
                        interfered = true;
                    }
                    return;
                }
            }

            else if (util::sign(l_error) != l_sgn || util::sign(r_error) != r_sgn) {
                if (print_toggle) std::cout << " Drive Wait Unitl Exit.\n";
                return;
            }
            
            pros::delay(util::DELAY_TIME);
        }
    }

    if (get_mode() == TURN || get_mode() == SWING || get_mode() == ARC) {
        int g_error = target - get_gyro();
        int g_sign = util::sign(g_error);

        exit_output turn_exit = RUNNING;
        exit_output swing_exit = RUNNING;

        pros::Motor& sensor = current_swing == util::LEFT_SWING ? left_motors[0] : right_motors[0];

        while (true) {
            g_error = target - get_gyro();

            if (get_mode() == TURN) {
                if (turn_exit == RUNNING) {
                    turn_exit = turn_exit != RUNNING ? turn_exit : turnPID.exit_condition({left_motors[0], right_motors[0]});
                    pros::delay(util::DELAY_TIME);
                } else {
                    if (print_toggle) std::cout << "  Turn; " << exit_to_string(turn_exit) << " Wait Until Exit.\n";

                    if (turn_exit == mA_EXIT || turn_exit == VELOCITY_EXIT) {
                        interfered = true;
                    }
                    return;
                }
            }

            else {
                if (util::sign(g_error) == g_sign) {
                    if (swing_exit == RUNNING) {
                        swing_exit = swing_exit != RUNNING ? swing_exit : swingPID.exit_condition(sensor);
                        pros::delay(util::DELAY_TIME);
                    } else {
                        if (print_toggle) std::cout << " Swing: " << exit_to_string(swing_exit) << " Wait Until Exit.\n";

                        if (swing_exit == mA_EXIT || swing_exit == VELOCITY_EXIT) {
                            interfered = true;
                        }
                        return;
                    }
                }

                else if (util::sign(g_error) != g_sign) {
                    if (print_toggle) std::cout << " Swing Wait Until Exit.\n";
                    return;
                }
            }

            pros::delay(util::DELAY_TIME);
        }
    }
}

