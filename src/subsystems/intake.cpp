#include "main.h"
#include "pros/motors.h"
#include "subsystems/intake.hpp"
#include "util.hpp"
#include <vector>

Intake::Intake(std::vector<int> intakePorts, pros::motor_gearset_e_t cart){
  for(auto i : intakePorts) {
    pros::Motor temp(abs(i), cart, is_reversed(i));
    intake.push_back(temp);
  }
}

void Intake::spin(int speed) {
  for (auto i : intake) {
    i.move(speed);
  }
}