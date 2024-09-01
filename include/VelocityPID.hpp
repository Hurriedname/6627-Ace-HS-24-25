#pragma once
#include "api.h"
#include "util.hpp"

class velPID {

    public:

        /**
         * \brief Default Constructor
         */
        velPID();

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
        velPID(double p, double i = 0, double d = 0, double f = 0);

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
        void set_constants(double p, double i = 0, double d = 0, double f = 0);

        /**
         * \brief Struct for constants
         */
        struct Constants{
            double kp;
            double ki;
            double kd;
            double kf;
        }constants;


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


};