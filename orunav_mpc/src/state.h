#pragma once

/******************************************
 **************** INCLUDES ****************
 ******************************************/

#include "commonDefines.h"


/******************************************/
/** 
 * @brief Defines the state of a car-like vehicle 
 */
class State
{
    public:
        double raw[SW_STATE_VAR_N];


        State();

        void set(const double, const double, const double, const double);


        double &x ();
        const double &x () const;

        double &y ();
        const double &y () const;
        
        double &theta ();
        const double &theta () const;

        double &phi ();
        const double &phi () const;


        void transform(const State &, const double, const double);

        const State operator-(const State&) const;
        State & operator-=(const State&);
        void feedbackError(const State &, const State &, const State &);

        void sqrt();
        void normalizeTheta ();
        void normalizePhi ();

        friend ostream& operator<< (ostream&, const State&);
};


class concatState
{
    public: 
        double *raw;


        concatState(double *);

        double &x (const unsigned int);
        const double &x (const unsigned int) const;

        double &y (const unsigned int);
        const double &y (const unsigned int) const;
        
        double &theta (const unsigned int);
        const double &theta (const unsigned int) const;

        double &phi (const unsigned int);
        const double &phi (const unsigned int) const;


        void multiply (const unsigned int, const State &);
};
