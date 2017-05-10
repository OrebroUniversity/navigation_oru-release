#pragma once

#include <stdexcept>
#include <string>
#include <stdint.h>

#include "config.h"

using namespace std;


#define SW_THROW_MSG(s) throw runtime_error(string("In ") + __func__ + "() // " + (s))


// Constants
const double SW_PI = 3.14159265358979323846;

const double SW_SAMPLING_PERIOD_SEC              = 0.060;
const double SW_SAMPLING_PERIOD_MICROSEC        = 60000;


// MPC
const unsigned int SW_CONTROL_VAR_N      = 2;
const unsigned int SW_STATE_VAR_N        = 4;
const unsigned int SW_STATE_CONCAT_LEN   = SW_STATE_VAR_N*SW_PREVIEW_WINDOW_LEN;
const unsigned int SW_CONTROL_CONCAT_LEN = SW_CONTROL_VAR_N*SW_PREVIEW_WINDOW_LEN;
/// Extended preview window also includes the preceding trajectory step.
const unsigned int SW_PREVIEW_WINDOW_LEN_EXT = SW_PREVIEW_WINDOW_LEN + 1;


// These types are used for interation with other ROS mudules.
typedef int32_t trajectoryID;

/*
 ****************************************************************************
 * The following constants are local for the respective files and should not 
 * be used elsewhere. They are gathered here for convenience.
 ****************************************************************************
 */

//---------------------
// canlibWrapper
const unsigned int SW_CAN_CHANNEL = 0;
//---------------------


//---------------------
// commandSender
#ifndef SW_BUILD_SIMULATION
// CAN bus
const unsigned int SW_CAN_NODE_ID = 30;
const unsigned int SW_CAN_COMMAND_ID = 384;
const unsigned char SW_CAN_FLAG_OVERRIDE = 0x01;
const unsigned char SW_CAN_FLAG_DRIVE_ON = 0x02;
const unsigned char SW_CAN_FLAG_STEER_ON = 0x04;
const unsigned int SW_CAN_CLEAR_ITER_NUM = 100;
const unsigned int SW_CAN_CLEAR_DELAY = 10000; // microsecond
#endif // ! SW_BUILD_SIMULATION

/// Is used to convert m/s to mm/s
const unsigned int SW_VELOCITY_RESOLUTION = 1000;
/// Is used to convert radian to centidegrees
const double SW_STEER_RESOLUTION_PI = 180*100/SW_PI;
/// Is used to determine, when given command is negligible.
const double SW_VELOCITY_COMMAND_TOL = 1e-2;
//---------------------


//---------------------
// navserverConnection
#ifdef SW_ENABLE_NAVSERVER_READER
const char SW_NAVSERVER_SUBSCRIBE[] = "subscribe state enc\r\n";
const char SW_NAVSERVER_STOP[]      = "!\r\n";
const char SW_NAVSERVER_IP[]        = "192.168.100.100";
const unsigned short int SW_NAVSERVER_PORT      = 5432;
const unsigned int SW_NAVSERVER_READ_BUF_SIZE   = 512;
const unsigned int SW_NAVSERVER_SELECT_TIMEOUT  = 80000; // microsecond
const unsigned int SW_NAVSERVER_BUF_CLEAR_ITER  = 10;
#endif
//---------------------


//---------------------
// qpConstraints
/// Tolerance for the angular velocity of a car.
/// (Used for computation of constraints on centripetal acceleration).
const double SW_ANGULAR_VELOCITY_TOL = 1e-2;
//---------------------


//---------------------
// qpProblem
/// To compensate for delays in the servo use a future control value (v/w).
/// The delay corresponds to [SW_SAMPLING_PERIOD*OFFSET].
/// (Only used if the SW_CITITRUCK_TIME_OFFSET is set).
const double SW_CITITRUCK_TIME_OFFSET_V = 3;
const double SW_CITITRUCK_TIME_OFFSET_W = 3;
//---------------------


//---------------------
// ROSSensorSubscriber
const unsigned int SW_SIM_UPDATES_PER_SAMPLING_PERIOD = 60;
const double SW_SIM_CALLBACK_WAIT_SEC = 0.2;
//---------------------


//---------------------
// SnowWhite
const unsigned int SW_THREAD_WAIT_TIME = 500000;
//---------------------


//---------------------
// swLogger
#ifdef SW_ENABLE_LOGGING
const char SW_ROBOT_ID_PREFIX_LOG_FILE[] = "robot";
const char SW_SENSOR_LOG_FILE[]          = "sensor.log";
const char SW_CONTROLLER_LOG_FILE[]      = "controller.log";
const char SW_TRAJECTORY_LOG_FILE[]      = "trajectory.log";
const char SW_EVALUATION_LOG_FILE[]      = "evaluation.log";

const char QP_LOG_FILE[]                 = "failed_qp.log";
#endif
//---------------------


//---------------------
// trajectoryCache
/// Must be >= #SW_PREVIEW_WINDOW_LEN_EXT.
const unsigned int SW_TRAJECTORY_CACHE_LEN = SW_PREVIEW_WINDOW_LEN_EXT;
//---------------------


//---------------------
// threadPool
const unsigned int SW_SCHED_PRIORITY = 50;
//---------------------


//---------------------
// trajectoryReader
const double SW_TRAJECTORY_CALLBACK_WAIT_SEC = SW_SAMPLING_PERIOD_SEC;
const double SW_HEARBEAT_TIMEOUT_SEC = 2.0;
const double SW_START_TIME_ALLOWED_ERROR_SEC = 0.1;
//---------------------


//---------------------
// canSensorReader
#ifndef SW_ENABLE_NAVSERVER_READER
const unsigned int SW_CAN_READ_TIMEOUT_MS = 80;
const int SW_CAN_MSG_POSITION_ID = 542;
const int SW_CAN_MSG_THETA_PHI_VEL_ID = 798;
#endif
//---------------------
