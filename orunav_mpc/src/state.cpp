#include <iostream>
#include <iomanip>
#include <cmath>

#include "state.h"


/**
 * @brief Constructor.
 */
State::State()
{
    set (0.0, 0.0, 0.0, 0.0);
}


/**
 * @brief Set the state variables.
 *
 * @param[in] x_ coordinate x.
 * @param[in] y_ coordinate y.
 * @param[in] theta_ orientation of the car.
 * @param[in] phi_ steering angle.
 */
void State::set(
        const double x_, 
        const double y_, 
        const double theta_, 
        const double phi_)
{
    x()     = x_;
    y()     = y_;
    theta() = theta_;
    phi()   = phi_;
}


///@{
/// Access to the coordinate x.
double &State::x  () {return raw[0];}
const double &State::x  () const {return raw[0];}
///@}

///@{
/// Access to the coordinate y.
double &State::y  () {return raw[1];}
const double &State::y  () const {return raw[1];}
///@}

///@{
/// Access to the orientation of the car.
double &State::theta  () {return raw[2];}
const double &State::theta  () const {return raw[2];}
///@}

///@{
/// Access to the steering angle.
double &State::phi  () {return raw[3];}
const double &State::phi  () const {return raw[3];}
///@}


/**
 * @brief Represent the state in the reference frame associated with some other state.
 *
 * @param[in] origin reference state.
 * @param[in] sinT sin(theta) in the reference state.
 * @param[in] cosT cos(theta) in the reference state.
 */
void State::transform(const State &origin, const double sinT, const double cosT)
{
    x()     -= origin.x();
    y()     -= origin.y();

    theta() = theta() - origin.theta();
    normalizeTheta();

    double x_copy = x();
    x() =  cosT*x()     + sinT*y();
    y() = -sinT*x_copy  + cosT*y();
}


/**
 * @brief Find the difference between two states.
 *
 * @param[in] sub subtract this state.
 *
 * @return result of the subtraction.
 */
const State State::operator-(const State& sub) const 
{
    State res;

    res.x()     = x()     - sub.x();
    res.y()     = y()     - sub.y();
    res.theta() = theta() - sub.theta();
    res.normalizeTheta();
    res.phi()   = phi()   - sub.phi();
    res.normalizePhi();

    return (res);
}


/**
 * @brief Find the difference between two states.
 *
 * @param[in] sub subtract this state.
 *
 * @return result of the subtraction.
 */
State & State::operator-=(const State& sub) 
{
    x()     -= sub.x();
    y()     -= sub.y();
    theta() -= sub.theta();
    normalizeTheta();
    phi()   -= sub.phi();
    normalizePhi();

    return (*this);
}


/** 
 * @brief Correct a state using error feedback: state = expected_state + gains*(sensor_state - expected_state).
 *              
 * @param[in] expected_state an expected state, which correspond to the state 
 *                           obtained using sensors
 * @param[in] sensor_state a state read from sensors
 * @param[in] gains gains
 */
void State::feedbackError(
        const State &expected_state,
        const State &sensor_state,
        const State &gains)
{       
    x()     = expected_state.x()     + gains.x()     * (sensor_state.x()     - expected_state.x());
    y()     = expected_state.y()     + gains.y()     * (sensor_state.y()     - expected_state.y());
    theta() = expected_state.theta() + gains.theta() * (sensor_state.theta() - expected_state.theta());
    phi()   = expected_state.phi()   + gains.phi()   * (sensor_state.phi()   - expected_state.phi());

    normalizeTheta();
    normalizePhi();
}



/**
 * @brief Finds square root of each variable in the state.
 */
void State::sqrt()
{
    x()     = std::sqrt(x());
    y()     = std::sqrt(y());
    theta() = std::sqrt(theta());
    phi()   = std::sqrt(phi());
}


/**
 * @brief Normalize theta (-pi,pi).
 */
void State::normalizeTheta ()
{
    theta() = atan2(sin(theta()),cos(theta()));
}


/**
 * @brief Normalize phi (-pi,pi).
 */
void State::normalizePhi ()
{
    phi() = atan2(sin(phi()),cos(phi()));
}


/**
 * @brief Output function.
 *
 * @param[in,out] out output stream.
 * @param[in] state the state.
 *
 * @return output stream.
 */
ostream& operator<< (ostream& out, const State& state) 
{
    out << setiosflags(ios::fixed) << setprecision(3)
        << "x = "        << setw(8) <<  state.x()
        << ",  y = "     << setw(8) <<  state.y()
        << ",  theta = " << setw(8) <<  state.theta()
        << ",  phi = "    << setw(8) <<  state.phi();
    return(out);
}




concatState::concatState(double *raw_ptr)
{
    raw = raw_ptr;
}


///@{
/// Access to the coordinate x.
double &concatState::x  (const unsigned int i) {return raw[SW_STATE_VAR_N*i + 0];}
const double &concatState::x  (const unsigned int i) const {return raw[SW_STATE_VAR_N*i + 0];}
///@}

///@{
/// Access to the coordinate y.
double &concatState::y  (const unsigned int i) {return raw[SW_STATE_VAR_N*i + 1];}
const double &concatState::y  (const unsigned int i) const {return raw[SW_STATE_VAR_N*i + 1];}
///@}

///@{
/// Access to the orientation of the car.
double &concatState::theta  (const unsigned int i) {return raw[SW_STATE_VAR_N*i + 2];}
const double &concatState::theta  (const unsigned int i) const {return raw[SW_STATE_VAR_N*i + 2];}
///@}

///@{
/// Access to the steering angle.
double &concatState::phi  (const unsigned int i) {return raw[SW_STATE_VAR_N*i + 3];}
const double &concatState::phi  (const unsigned int i) const {return raw[SW_STATE_VAR_N*i + 3];}
///@}


void concatState::multiply (const unsigned int i, const State & state)
{
    x(i)        *= state.x();
    y(i)        *= state.y();
    theta(i)    *= state.theta();
    phi(i)      *= state.phi();
}
