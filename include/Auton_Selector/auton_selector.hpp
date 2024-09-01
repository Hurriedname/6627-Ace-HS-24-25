#pragma once
#include <tuple>

#include "auton.hpp"

using namespace std;

class AutonSelector {
    public:
    std::vector<Auton> Autons;
    int current_auton_page;
    int auton_count;
    AutonSelector();
    //lvgl_Auton();
    AutonSelector(std::vector<Auton> autons);
    void call_selected_auton();
    void print_selected_auton();
    void add_autons(std::vector<Auton> autons);
    
};
/*class LVGLAUTO {
    public:
    std::vector<Auton> Autons;
    int current_auton_page;
    int auton_count;
    AutonSelector();
    AutonSelector(std::vector<Auton> autons);
    void call_selected_auton();
    void print_selected_auton();
    void add_autons(std::vector<Auton> autons);
    
};*/
namespace as {
    extern AutonSelector auton_selector;

    void init_auton_selector();
    void page_up();
    void page_down();
    void initialize();
    void shutdown();
}
