#include "chassis.hpp"
#include "main.h"
#include "pros/misc.h"
#include "util.hpp"

void Chassis::set_curve_default(double left, double right) {
    left_curve_scale = left;
    right_curve_scale = right;
}

void Chassis::set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
    l_increase_.button = increase;
    l_decrease_.button = decrease;
}

void Chassis::set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase) {
    r_increase_.button = increase;
    r_decrease_.button = decrease;
}

void Chassis::l_increase() { left_curve_scale += 0.1; }
void Chassis::l_decrease() {
    left_curve_scale -= 0.1;
    left_curve_scale = left_curve_scale < 0 ? 0 : left_curve_scale;
}

void Chassis::r_increase() { right_curve_scale += 0.1; }
void Chassis::r_decrease() {
    right_curve_scale -= 0.1;
    right_curve_scale = right_curve_scale < 0 ? 0: right_curve_scale;
}

void Chassis::button_press(button_* input_name, int button, std::function<void()> change_curve) {

    if (button && !input_name->lock) {
        change_curve();
        input_name->lock = true;
        input_name->release_reset = true;
    }

    else if (button && input_name->lock) {
        input_name->hold_timer += util::DELAY_TIME;
        if (input_name->hold_timer > 500.0) {
            input_name->increase_timer += util::DELAY_TIME;
            if (input_name->increase_timer > 100.0) {
                change_curve();
                input_name->increase_timer = 0;
            }
        }
    }

    else if (!button) {
        input_name->lock = false;
        input_name->hold_timer = 0;

        if (input_name->release_reset) {
            input_name->release_timer += util::DELAY_TIME;
            if (input_name->release_timer > 250.0) {
                input_name->release_timer = 0;
                input_name->release_reset = false;
            }
        }
    }
}

void Chassis::toggle_modify_curve_with_controller(bool toggle) { disable_controller = toggle; }

void Chassis::modify_curve_with_controller() {
    if (!disable_controller) return;

    button_press(&l_increase_, master.get_digital(l_increase_.button), ([this] { this->l_increase(); }) );
    button_press(&l_decrease_, master.get_digital(l_decrease_.button), ([this] { this->l_decrease(); }) );

    if (!is_tank) {
    button_press(&r_increase_, master.get_digital(r_increase_.button), ([this] { this->r_increase(); }) );
    button_press(&r_decrease_, master.get_digital(r_decrease_.button), ([this] { this->r_decrease(); }) );  
    }

    auto sr = std::to_string(right_curve_scale);
    auto sl = std::to_string(left_curve_scale);

    if (!is_tank)   master.set_text(2, 0, sl + "    " + sr);
    else    master.set_text(2, 0, sl);
}

double Chassis::left_curve_function(double x) {
    if (left_curve_scale != 0) {
        return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
    }
    return x;
}

double Chassis::right_curve_function(double x) {
    if (right_curve_scale != 0) {
        return (powf(2.718, -(right_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(right_curve_scale / 10)))) * x;
    }
    return x;    
}

void Chassis::set_joystick_threshold(int threshold) {   JOYSTICK_THRESHOLD = abs(threshold); }

void Chassis::reset_drive_sensors_opcontrol() {
    if (util::AUTON_RAN) {
        reset_drive_sensors();
        util::AUTON_RAN = false;
    }
}

//// Possible issue without the active brake
void Chassis::joy_thresh_opcontrol(int l_stick, int r_stick) {
    if (abs(l_stick) > JOYSTICK_THRESHOLD || abs(r_stick) > JOYSTICK_THRESHOLD) {
        set_tank(l_stick, r_stick);
        util::DRIVING = true;
    }
    else {
        util::DRIVING = false;
        set_tank(0, 0);
    }
}

void Chassis::arcade(e_type stick_type) {
    is_tank = false;
    reset_drive_sensors_opcontrol();

    modify_curve_with_controller();

    int turn_stick, fwd_stick;

    if (stick_type == SPLIT) {
        turn_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        fwd_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    }

    joy_thresh_opcontrol(fwd_stick + turn_stick, fwd_stick - turn_stick);
}
void Chassis::arcade_flipped_curvature(e_type stick_type) {
    is_tank = false;
    reset_drive_sensors_opcontrol();

    modify_curve_with_controller();

    double turn_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    double fwd_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    double left_stick = fwd_stick + turn_stick;
    double right_stick = fwd_stick - turn_stick;
    double mag = std::max(fabs(left_stick), fabs(right_stick)) / 127.0;

    if (mag > 1.0) {
      left_stick /= mag;
      right_stick /= mag;
    }
    joy_thresh_opcontrol((left_stick), (right_stick));
}

void Chassis::tank() {
    is_tank = true;
    reset_drive_sensors_opcontrol();
    modify_curve_with_controller();

    int left_stick = left_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    int right_stick = right_curve_function(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

    joy_thresh_opcontrol(left_stick, right_stick);
}

