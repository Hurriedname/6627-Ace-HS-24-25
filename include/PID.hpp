#pragma once
#include "api.h"
#include "util.hpp"

class PID {

    public:

        /**
         * \brief Default Constructor
         */
        PID();

        /**
         * \brief Constructor with constants
         *
         * \param p
         *        Kp
         * \param i
         *        Ki
         * \param d
         *        Kd
         * \param start_i
         *        error value that i starts within
         * \param name
         *        std::string of name that prints
         */
        PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");

        /**
         * \brief Constructor with constants
         *
         * \param p
         *        Kp
         * \param i
         *        Ki
         * \param d
         *        Kd
         * \param start_i
         *        error value that i starts within
         */
        void set_constants(double p, double i = 0, double d = 0, double _start_i = 0);

        /**
         * \brief Struct for constants
         */
        struct Constants{
            double kp;
            double ki;
            double kd;
            double start_i;
        }constants;

        /**
         * \brief Struct for exit conditions
         */
        struct _exit_condition {
            int small_exit_time = 0;
            double small_error = 0;
            int big_exit_time = 0;
            double big_error = 0;
            int velocity_exit_time = 0;
            int mA_timeout = 0;
        }exit;

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
        void set_exit_condition(double _small_error, int _small_exit_time, double _big_error = 0, int _big_exit_time = 0, int _velocity_exit_time = 0, int _mA_timeout = 0);

        /**
         * \brief sets PID target
         *
         * \param input
                  target for PID
         */
        void set_target(double input);

        /**
         * \brief computes PID
         *
         * \param input
         *        current sensor reading
         */
        double compute(double input);

        /**
         * \brief returns_target
         */
        double return_target();

        /**
         * \brief return constants
         */
        Constants return_constants();

        /**
         * \brief resets PID variables
         */
        void reset_variables();

        /**
         * \brief exit condition
         *
         * \param print
         *        print the exit condition type
         */
        util::exit_output exit_condition(bool print = false);

        /**
         * \brief exit condition
         *
         *
         *\param sensor
         *       determines which motor to detect the mA from
         *
         * \param print
         *        print the exit condition type
         */
        util::exit_output exit_condition(pros::Motor sensor, bool print = false);

        /**
         * \brief exit condition
         *
         *
         *\param sensor
         *       determines which motors to detect the mA from
         *
         * \param print
         *        print the exit condition type
         */
        util::exit_output exit_condition(std::vector<pros::Motor> sensor, bool print = false);

        /**
         * \brief sets PID name
         *
         * \param name
         *        string for the name of the pid object
         */
        void set_name(std::string name);

        /**
         * \brief PID variables
         */
        double proportion;
        double integral;
        double derivative;
        double output;
        double actual;
        double target;
        double error;
        double prev_error;
        long time;
        long prev_time;

    private:
        int big_timer = 0, small_timer = 0, vel_timer = 0, current_timer = 0;
        bool is_mA = false;
        void reset_timers();
        std::string name;
        bool is_name = false;
        void print_exit(util::exit_output exit_type);

};