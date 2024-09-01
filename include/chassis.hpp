#pragma once
#include "api.h"
#include <functional>
#include <iostream>
#include <tuple>
#include <vector>

#include "PID.hpp"
#include "VelocityPID.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "util.hpp"
#include "pros/motors.h"


using namespace util;

class Chassis {

    public:

    /**
     * \brief threshold for joystick (returns zero when joystick is whithin this number)
     */
    int JOYSTICK_THRESHOLD;

    /**
     * Global current brake mode
     */
    pros::motor_brake_mode_e_t CURRENT_BRAKE = pros::E_MOTOR_BRAKE_COAST;

    /**
     * Global current mA
     */
    int CURRENT_MA = 2500;

    /**
     * Current swing type
     */
    e_swing current_swing;

    /**
     * \brief Vector of motors for left and right side of the chassis
     */
    std::vector<pros::Motor> left_motors;
    std::vector<pros::Motor> right_motors;

    /**
     * \brief Vector of pros motors that are disconnected from the drive
     */
    std::vector<int> pto_active;

    /**
     * \brief Inertial Sensor
     */
    pros::Imu imu;

    /**
     * \brief Tracking Wheels
     */
    
    
    /*pros::ADIEncoder l_enc {'E','F', false};
    pros::ADIEncoder r_enc {'C','D', true};
    */
    //pros::ADIEncoder vertical_enc {'A','B', false};
    //pros::ADIEncoder horizontal_enc {'C','D', true};
    
    /**
     * \brief PID objects
     */

    PID forward_drivePID;
    PID backward_drivePID;  
    PID leftPID;
    PID rightPID;
    PID linearPID;
    PID angularPID;
    PID headingPID;   
    PID swingPID;
    PID turnPID;
    PID arcPID;
    velPID rightVelocityPID;
    velPID leftVelocityPID;

    /** 
     * \brief Current mode of the drive
     */
    e_mode mode;

    /**
     * \brief sets mode the drive
     */
    void set_mode(e_mode _mode);

    /**
     * \brief current mode of drive
     */
    e_mode get_mode();

    /**
     * \brief Calibrates imu
     */
    void initialize();

    /**
     * \brief async task for auton
     */
    pros::Task async_auto;

    /**
     * \brief tracking task for auton
     */
    pros::Task tracking;


    /**
     * \brief Drive Controller using three encoders blugged into the brain.
     *
     * \param left_motor_ports
     *        Input {1, -2...}.  Make ports negative if reversed!
     * \param right_motor_ports
     *        Input {-3, 4...}.  Make ports negative if reversed!
     * \param imu_port
     *        Port the IMU is plugged into.
     * \param wheel_diameter
     *        Diameter of your sensored wheels.  Remember 4" is 4.125"!
     * \param ticks
     *        Ticks per revolution of your encoder.
     * \param ratio
     *        External gear ratio, wheel gear / sensor gear.
     */
    Chassis(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports, int imu_port, double wheel_diameter, double ticks, double ratio, pros::motor_gearset_e_t cart, double chassis_width, double left_offset, double right_offset, double center_offset);

    /**
     * \brief Sets drive details
     */
    void set_defaults();


    /**
     * \brief User Control
     */

    /**
     * \brief sets the cahssis to controller joysticks using tank or arcade control. Run is usercontrol
     */
    void tank();  
    void arcade(e_type stick_type);
    void arcade_flipped_curvature(e_type stick_type);


    /**
     * \brief sets the default joystick cruves
     */
    void set_curve_default(double left, double right = 0);

    /**
     * Runs a P loop on the drive when the joysticks are released.
     *
     * \param kp
     *        Constant for the p loop.
     */
    void set_active_brake(double kp);

    /**
     * \brief toggle for modifying  input curves with the controller
     *
     * \param input
     *        bool input
     */
    void toggle_modify_curve_with_controller(bool toggle);

    /**
     * \brief sets buttons for modifying the left and right joystick curve
     *
     * \param decrease
     *        a pros button enumerator
     * \param increase
     *        a pros button enumerator
     */
    void set_left_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);
    void set_right_curve_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

    /**
     * \brief Outputs curve from 5225A (Allows for more controll over the robot at lower speeds)
     *
     * \param x
              joystick input
     *
     */
    double left_curve_function(double x);
    double right_curve_function(double x);

    /**
     * \breif sets joystick threshold
     *
     * \param threshold
     *        threshold value
     */
    void set_joystick_threshold(int threshold);

    /**
     * \brief resets drive sensors at the start of opcontrol
     */
    void reset_drive_sensors_opcontrol();

    /** 
     * \brief joystick threhold at the start of op control
     *
     * \param l_stick
     *        input for left joystick
     * \param r_stick
     *        input for right joystick
     */
    void joy_thresh_opcontrol(int l_stick, int r_stick);


    /**
     * \brief PTO
     */
    
    /**
     * \brief checks if motor is in pto_list
     *
     * \param check_if_pto
     *        THe motor to check
     */
    bool pto_check(pros::Motor check_if_pto);

    /**
     * \brief adds motor to pot list and removes them from the drive
     *
     * \param pto_list
              list of motors to remove from the drive
     */
    void pto_add(std::vector<pros::Motor> pto_list);

    /**
     * \brief removes motor to pot list and adds them from the drive
     *
     * \param pto_list
     *        list of motors to add tp the drive
     */
    void pto_remove(std::vector<pros::Motor> pto_list);

    /**
     * \brief adds/removes motors from drive.
     *
     * \param pto_list
     *        list of motors to add/remove from the drive
     * \param toggle
     *        if true, adds to list, if false
     */
    void pto_toggle(std::vector<pros::Motor> pto_list, bool toggle);


    /**
     * \brief PROS Wrappers
     */

    /**
     * \brief Sets chassis to voltage
     *
     * \param left
     *        voltage to left side, -127 to 127
     * \param right
     *        voltage to right side, -127 to 127
     */
    void set_tank(int left, int right);

    void set_tank_velocity(int left, int right);

    /**
     * \brief changes the brake type of the drive
     *
     * \param brake_type
     *        sets the brake mode of the motor (coast, brake, hold)
     */
    void set_drive_brake(pros::motor_brake_mode_e_t brake_type);

    /**
     * \brief Sets the limit for the current on the drive
     *
     * \param mA
     *        input in milliamps
     */
    void set_drive_current_limit(int mA);

    /**
     * \brief toggle set drive in auton
     *
     * \param toggle
     *        true enables, false disables
     */
    void toggle_auto_drive(bool toggle);

    /**
     * \brief toggles printin in auton
     *
     * \param toggle
     *        true enables, false disables
     */
    void toggle_auto_print(bool toggle);

    void toggle_auto_graph_var(bool toggle);

    /**
     * \brief the position of the encoders
     */
    int left_position();
    int right_position();
    double vertical_position();
    double horizontal_position();

    /**
     * \brief velocity of motor
     */
    int right_velocity();
    int left_velocity();

    /**
     * \brief The watts of the motor
     */
    double right_mA();
    double left_mA();

    /**
     * \brief returns true if motor is over current
     */
    bool right_over_current();
    bool left_over_current();

    /**
     * \brief resets all encoders
     */
    void reset_drive_sensors();

    /**
     * \brief resets current gyro heading
     *
     * \param new_heading
     *        new heading value
     */
    void reset_gyro(double new_heading = 0);

    /**
     * \brief returns the current gyro value
     */
    double get_gyro();

    /**
     * \brief Calibrates the IMU
     */
    bool imu_calibrate();


    /**
     * \brief Autonomous Functions
     */
    
    /**
     * \brief sets the robot to move forward using PID
     *
     * \param target
     *        target value in inches
     * \param speed
     *        0 to 127, max speed motors can move at
     * \param slew_on
     *        ramp up from slew_min to speed over slew_distance
     * \param toggle_heading
     *        toggle for heading correction
     */
    void set_drive_pid(double target, int speed, bool slew_on = false, bool toggle_heading = true);

    /**
     * \brief sets the robot to move to a set point using PID
     */
    void move_to_point(e_dir direction, double x_target, double y_target, int speed, bool slew_on = false, double deadband = 0, bool turn_scale_on = true);

    /**
     * \brief sets the robot to turn to a set point using PID
     */
    void turn_to_point(e_dir direction, double x_target, double y_target, int speed);
    
    /**
     * \brief sets robot to move in a circular arc using PID
     */
    void set_arc_pid(e_dir direction, double target, double circle_radius, int speed);

    /**
     * \brief sets the robot to turn using PID
     *
     * \param target
     *        target vakue in degrees
     * \param speed
     *        0 to 127, max speed during motion
     */
    void set_turn_pid(double target, int speed);

    /**
     * \brief turns by only using left or right side
     *
     * \param type
     *        left swing or right swing
     * \param target
     *        target value in degrees
     * \param speed
     *        0 to 127, max speed during auton
     */
    void set_swing_pid(e_swing type, double target, int speed);

    /**
     * \brief Resets all PID targets to 0
     */
    void reset_pid_targets();

    /**
     * \brief sets the angle
     */
    void set_angle(double angle);

    /**
     * \brief holds the program until the drive has settled
     */
    void wait_drive();

    /**
     * \brief holds the program until the arget distance is met
     *
     * \param target
     *        when driving this is in inches. when turning this is in degrees
     */
    void wait_until(double target);

    /**
     * \brief Auton interferance detection
     */
    bool interfered = false;

    /**
     * \brief changes max speed during drive motion
     *
     * \param speed
     *        new clipped speed
     */
    void set_max_speed(int speed);

    /**
     * \brief Set either the headingPID, turnPID, forwardPID, backwardPID, activeBrakePID, or swingPID
     */
    void set_pid_constants(PID *pid, double p, double i, double d, double _start_i);

    /**
     * \brief sets min power for swings when Ki and startI are enabled
     *
     * \param min
     *        new clipped speed
     */
    void set_swing_min(int min);

    /**
     * \brief sets min power for turn when Ki and startI are enabled
     *
     * \param min
     *        new clipped speed
     */
    void set_turn_min(int min);

    /**
     * \brief returns min power for swings when Ki and startI are enabled
     */
    int get_swing_min();

    /**
     * \brief returns min power for turns when Ki and startI are enabled
     */
    int get_turn_min();

    /**
     * \brief sets min slew speed constnats
     *
     * \param fwd
     *        min power for fwd drive pd
     * \param rev
     *        min power for bwd drive pd
     */
    void set_slew_min_power(int fwd, int rev);

    /**
     * \brief Sets minimum slew distance constants.
     *
     * \param fw
     *        minimum distance for forward drive pd
     * \param bw
     *        minimum distance for backwards drive pd
     */
    void set_slew_distance(int fwd, int rev);

    /**
     * \brief Sets constants for exit conditions
     *
     * \param _small_error
     *        Sets small error
     * \param _small_exit_time
     *      sets small exit time. Time starts when error is within small_error
     * \param _big_error
     *        sets the big error
     * \param _big_exit_time
     *        sets small exit time. Time starts when error is within big_error
     * \param _velocity_exit_time
     *        sets the velocity exit time. Time starts when velocity is zero
     * \param _mA_timeout
     *        sets the mA timeout. Time starts when current over the max current
     */
    void set_exit_condition(int type, double _small_error, int _small_exit_time, double _big_error = 0, int _big_exit_time = 0, int _velocity_exit_time = 0, int _mA_timeout = 0);

    /**
     * \brief Exit condition for turning, swinging, and driving
     */
    const int turn_exit = 1;
    const int swing_exit = 2;
    const int drive_exit = 3;
    const int linear_exit = 4;
    const int angular_exit = 5;
    const int arc_exit = 6;
    

    /**
     * \brief returns current ticks per inch
     */
    double get_tick_per_inch();

    /**
     * \brief modify curve with controller
     */
    void modify_curve_with_controller();


    ///
    // Odom
    ///

    /**
     * \brief Coordinates
     */
    typedef struct pose {
        double x;
        double y;
        double theta;
        double angleRad;
        double angleDeg;
    } pose;
    pose target;
    pose current;

    int t = 0;
    int initxErrorSign, inityErrorSign;

    double circleRadius = 0;
    double CHASSIS_WIDTH = 11.65;


    bool is_driver = false;
    e_dir current_direction;
    double deadband = 0;

    bool turn_scale_on = false;
    double InitialAngleError = 0;

    
    bool driverSkills = false;
    void print_pose();
    void set_x(double x);
    void set_y(double y);
    void set_theta(double a);
    void reset_odom();
    void set_pose(double x, double y, double a);
    pose projected_point(double distance, double xTarget, double yTarget, double theta, e_dir direction);


    /**
     * \brief Slew
     */
    struct slew_ {
        int sign = 0;
        double error = 0;
        double x_intercept = 0;
        double y_intercept = 0; 
        double slope = 0;
        double output = 0;
        bool enabled = false;
        double max_speed = 0;
    };
    slew_ left_slew;
    slew_ right_slew;
    slew_ vertical_slew;

    /**
     * \brief initialize slew
     *
     * \param input
     *        slew_ enum
     * \param slew_on
     *        is slew on?
     * \param max_speed
     *        target speed during the slew
     * \param target
     *        target sensor value
     * \param current
     *        crrent sensor value
     * \param start
     *        starting position
     * \param backwards
     *        slew direction for constants
     */
    void slew_initialize(slew_ &input, bool slew_on, double max_speed, double target, double current, double start, bool boackwards);

    /**
     * \brief Calculate slew
     *
     * \param input
     *        slew_ enum
     * \param current
     *        current sensor value
     */
    double slew_calculate(slew_ &input, double current);

    
    private:

    bool drive_toggle = true;
    bool print_toggle = true;
    bool graph_var_toggle = false;
    int swing_min = 0;
    int turn_min = 0;

    /**
     * \brief heading bool
     */
    bool heading_on = true;

    /**
     * \brief Tick per inch calculation
     */
    double TICKS_PER_REV;
    double TICKS_PER_INCH;
    double CIRCUMFERENCE;
    
    double CARTRIDGE;
    double RATIO;
    double WHEEL_DIAMETER;

    /**
     * \brief Max Speed for Auton
     */
    int max_speed;

    /**
     * \brief Auton Tasks
     */
    void drive_pid_task();
    void turn_pid_task();
    void swing_pid_task();
    void move_to_point_task();
    void angle_to_point_task();
    void arc_pid_task();
    void async_auto_task();
    void tracking_task();




    double LEFT_OFFSET;
    double RIGHT_OFFSET;
    double CENTER_OFFSET;    


    /**
     * \brief Slew Constants
     */
    double SLEW_DISTANCE[2];
    double SLEW_MIN_POWER[2];

    /**
     * \brief Starting value for left/right, x and y
     */
    double l_start = 0;
    double r_start = 0;
    double c_start = 0;
    double x_start = 0;
    double y_start = 0;

    /**
     * \brief enable/disable modifying controller curve with controllre
     */
    bool disable_controller = true;

    /**
     * \brief is tank running?
     */
    bool is_tank;

    #define DRIVE_INTEGRATED 1
    #define DRIVE_ADI_ENCODER 2

    /**
     * Is tracking?
     */
    int is_tracker = DRIVE_INTEGRATED;

    /**
     * \brief Struct for controller curve buttons
     */
    struct button_ {
        bool lock = false;
        bool release_reset = false;
        int release_timer = 0;
        int hold_timer = 0;
        int increase_timer;
        pros::controller_digital_e_t button;
    };

    button_ l_increase_;
    button_ l_decrease_;
    button_ r_increase_;
    button_ r_decrease_;

    /**
     * \brief Function for button presses
     */
    void button_press(button_ *input_name, int button, std::function<void()> changeCurve);

    /**
     * \brief the left and right curve scalers
     */
    double left_curve_scale;
    double right_curve_scale;

    /**
     * \brief Increase and decrease left and right curve scale
     */
    void l_decrease();
    void l_increase();
    void r_decrease();
    void r_increase();


};