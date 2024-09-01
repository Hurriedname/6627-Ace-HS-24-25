#pragma once
#include "api.h"
#include "util.hpp"
#include <functional>

extern pros::Controller master;


namespace util {
    
    extern bool AUTON_RAN;
    extern bool AUTON_IN_DRIVER;
    extern bool DRIVING;

    void print_to_screen(std::string text, int line = 0);
    void toggle_auton_in_driver(bool state);
    /**
     * \brief Public Variables
     */

    enum e_type { SINGLE = 0,
                   SPLIT = 1 };

    enum e_swing { LEFT_SWING = 0,
                   RIGHT_SWING = 1 };

    enum e_mode { 
                  DISABLE = 0,
                  SWING = 1,
                  TURN = 2, 
                  DRIVE = 3,
                  TO_POINT = 4,
                  ANGLE_TO_POINT = 5,
                  ARC = 6
                };

    enum e_dir { FWD = 0,
                 REV = 1,
                 LEFT = 2,
                 RIGHT = 3 };

    const int DELAY_TIME = 10;
    
    double to_deg(double rad);
    double to_rad(double deg);
    double wrap_angle_90(double theta);
    double wrap_angle_180(double theta);
    double wrap_angle_360(double theta);
    double hypot(double a, double b);
    double slope(double deltaX, double deltaY);
    double distance_to_point(double xTarget, double yTarget, e_dir direction);
    double angle_to_point(double xTarget, double yTarget, e_dir direction);


    
    void print_with_delay(std::function<void()> print, int interval);

    double clip_num(double input, double max, double min);
    
    enum exit_output { RUNNING = 1,
                       SMALL_EXIT = 2,
                       BIG_EXIT = 3,
                       VELOCITY_EXIT = 4,
                       mA_EXIT = 5,
                       ERROR_NO_CONSTANTS = 6 };

    int sign(double input);

    std::string exit_to_string(exit_output input);

    bool is_reversed(double input);

    class Test {

        public:
        
        std::vector<pros::Motor> test_motors;

        Test(std::vector<int> test_motor_ports);

        void benchmark();
    };
}

