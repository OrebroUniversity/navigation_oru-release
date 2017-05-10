#pragma once

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include <iostream>
#include <iomanip>

#include "commonDefines.h"


/**
 * @brief Control vector in the model
 */
class Control
{
    public:
        /// Raw data.
        double raw[SW_CONTROL_VAR_N]; 


        /**
         * @brief Constructor
         */
        Control()
        {
            set (0.0, 0.0);
        }


        /**
         * @brief Set variables.
         *
         * @param[in] v_ velocity (reference point).
         * @param[in] w_ angular velocity.
         */
        void set(const double v_, const double w_)
        {
            v() = v_;
            w() = w_;
        }

        ///@{
        /// Access to the velocity.
        double &v  () {return raw[0];};
        const double &v  () const {return raw[0];};
        ///@}

        ///@{
        /// Access to the angular velocity.
        double &w  () {return raw[1];}
        const double &w  () const {return raw[1];}
        ///@}


        /**
         * @brief Output function.
         *
         * @param[in,out] out output stream.
         * @param[in] control control.
         *
         * @return output stream.
         */
        friend ostream& operator<< (ostream& out, const Control& control)
        {
            out << setiosflags(ios::fixed) << setprecision(3)
                << "v = "       << setw(8) << control.v()
                << ", w = "     << setw(8) << control.w();
            return(out);
        }
};



class concatControl
{
    public:
        /// Raw data.
        double *raw;


        /**
         * @brief Constructor
         */
        concatControl(double *raw_ptr)
        {
            raw = raw_ptr;
        }



        ///@{
        /// Access to the velocity.
        double &v  (const unsigned int i) {return raw[i*SW_CONTROL_VAR_N + 0];};
        const double &v  (const unsigned int i) const {return raw[i*SW_CONTROL_VAR_N + 0];};
        ///@}

        ///@{
        /// Access to the angular velocity.
        double &w  (const unsigned int i) {return raw[i*SW_CONTROL_VAR_N + 1];}
        const double &w  (const unsigned int i) const {return raw[i*SW_CONTROL_VAR_N + 1];}
        ///@}
};


template <unsigned int size> class concatControlFixed : public concatControl
{
    public:
        double data[size * SW_CONTROL_VAR_N];

        concatControlFixed() : concatControl(data) {};
};
