
#include "main.h"
#include "pros/llemu.hpp"
#include "display/lvgl.h"

//AutonSelector::lvgl_Auton(){
    
//}


AutonSelector::AutonSelector() {
    auton_count = 0;
    current_auton_page = 0;
    Autons = {};
}

AutonSelector::AutonSelector(std::vector<Auton> autons) {
    auton_count = autons.size();
    current_auton_page = 0;
    Autons = {};
    Autons.assign(autons.begin(), autons.end());
}

void AutonSelector::print_selected_auton() {
    if (auton_count == 0) return;
        pros::lcd::clear_line(6);
        pros::lcd::set_text(6, Autons[current_auton_page].Name);
}

void AutonSelector::call_selected_auton() {
    if (auton_count == 0) return;
    Autons[current_auton_page].auton_call();
}

void AutonSelector::add_autons(std::vector<Auton> autons) {
    auton_count += autons.size();
    current_auton_page = 0;
    Autons.assign(autons.begin(), autons.end());
}


namespace as {
    AutonSelector auton_selector{};
    void page_up() {
        if (auton_selector.current_auton_page == auton_selector.auton_count - 1)
            auton_selector.current_auton_page = 0;
        else
            auton_selector.current_auton_page++;
        auton_selector.print_selected_auton();
    }

    void page_down() {
        if (auton_selector.current_auton_page == 0)
            auton_selector.current_auton_page = auton_selector.auton_count - 1;
        else
            auton_selector.current_auton_page--;
        auton_selector.print_selected_auton();
    }

    void initialize() {
        if (auton_selector.current_auton_page > auton_selector.auton_count - 1 || auton_selector.current_auton_page < 0)
            auton_selector.current_auton_page = 0;
        pros::lcd::initialize();

        as::auton_selector.print_selected_auton();
        pros::lcd::register_btn0_cb(as::page_down);
        pros::lcd::register_btn2_cb(as::page_up);
    }

    void shutdown() {
        pros::lcd::shutdown();
    }
}

