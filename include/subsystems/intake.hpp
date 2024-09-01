#pragma once
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include <vector>

using namespace util;

class Intake {
  public:

    Intake(std::vector<int> intakePorts, pros::motor_gearset_e_t cart);
    std::vector<pros::Motor> intake;

    void spin(int speed);

};
extern Intake intake;
