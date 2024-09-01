/*

#include <algorithm>
#include <vector>

#include "chassis.hpp"
#include "main.h"

bool Chassis::pto_check(pros::Motor check_if_pto) {
  auto does_exist = std::find(pto_active.begin(), pto_active.end(), check_if_pto.get_port());
  if (does_exist != pto_active.end())
    return true;  // Motor is in the list
  return false;   // Motor isn't in the list
}

void Chassis::pto_add(std::vector<pros::Motor> pto_list) {
  for (auto i : pto_list) {
    // Return if the motor is already in the list
    if (pto_check(i)) return;

    // Return if the first index was used (this motor is used for velocity)
    if (i.get_port() == left_motors[0].get_port() || i.get_port() == right_motors[0].get_port()) {
      printf("You cannot PTO the first index!\n");
      return;
    }

    pto_active.push_back(i.get_port());
  }
}

void Chassis::pto_remove(std::vector<pros::Motor> pto_list) {
  for (auto i : pto_list) {
    auto does_exist = std::find(pto_active.begin(), pto_active.end(), i.get_port());
    // Return if the motor isn't in the list
    if (does_exist == pto_active.end()) return;

    // Find index of motor
    int index = std::distance(pto_active.begin(), does_exist);
    pto_active.erase(pto_active.begin() + index);
    i.set_brake_mode(CURRENT_BRAKE);  // Set the motor to the brake type of the drive
    i.set_current_limit(CURRENT_MA);  // Set the motor to the mA of the drive
  }
}

void Chassis::pto_toggle(std::vector<pros::Motor> pto_list, bool toggle) {
  if (toggle)
    pto_add(pto_list);
  else
    pto_remove(pto_list);
}


*/



/*
#include <algorithm>
#include <vector>

#include "chassis.hpp"
#include "main.h"

bool Chassis::pto_check(pros::Motor check_if_pto) {
    auto does_exist = std::find(pto_active.begin(), pto_active.end(), check_if_pto.get_port());
    if (does_exist != pto_active.end())
        return true;
    return false;
}

void Chassis::pto_add(std::vector<pros::Motor> pto_list) {
    for (auto i : pto_list) {
        if (pto_check(i)) return;

        if (i.get_port() == left_motors[0].get_port() || i.get_port() == right_motors[0].get_port()) {
            printf("YOU cannot PTO the first index!\n");
            return;
        }

        pto_active.push_back(i.get_port());
    }
}

void Chassis::pto_remove(std::vector<pros::Motor> pto_list) {
    for (auto i : pto_list) {
        auto does_exist = std::find(pto_active.begin(), pto_active.end(), i.get_port());

        if (does_exist == pto_active.end()) return;

        int index = std::distance(pto_active.begin(), does_exist);
        pto_active.erase(pto_active.begin() + index);
        i.set_brake_mode(CURRENT_BRAKE);
        i.set_current_limit(CURRENT_MA);
    }
}

void Chassis::pto_toggle(std::vector<pros::Motor> pto_list, bool toggle) {
    if (toggle)
        pto_add(pto_list);
    else
        pto_remove(pto_list);
}
*/

