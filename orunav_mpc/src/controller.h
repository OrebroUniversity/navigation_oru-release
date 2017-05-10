#pragma once

#include "commonDefines.h"
#include "threadData.h"
#include "state.h"
#include "qpProblem.h"
#include "command.h"
#include "trajectoryCache.h"



/**
 * @brief A simple container, which stores states of a car. Estimated state 
 * should be used as the initial state of MPC problem.
 *
 * @swAssume Assumes that the expected steering angle is always reached. Hence,
 * the steering angle reported by sensors is ignored. If it is not ignored the
 * performance of the controller degrades due to a delay in sensor reports. The
 * duration of the delay is about 3 sampling periods (3*60 ms). Computation of
 * an error with 180ms delay does not make any improvement, most likely due to
 * the work of the onboard controller.
 */
class stateContainer
{
    public:
        stateContainer();
        void waitSensor(threadData *);
        const State & sensor() const;
        const State & expected() const;
        const State & estimated() const;
        void setExpected(const State &);


    private:
        /// Initialization flag
        bool is_initialized;
        /// The expected state returned by QP solver.
        State expected_state;
        /// The state determined using sensors.
        State sensor_state;
        /// Estimation of the current state of a car.
        State estimated_state;
};



/**
 * @brief Controller, which computes and sends commands.
 *
 * @swAssume Assumes, that all computations are completed before reception of the next 
 * synchronization signal. (Otherwise control iterations may be skipped).
 */
class Controller
{
    public:
        Controller();

        threadReturn controlLoop(threadData *);


    private:
        /// Trajectory cache
        trajectoryCache cache;

        stateContainer state;

        int graceful_counter;
        bool last_command_negligible;
        Control control;



        swProcessStatus getCommand(threadData *, qpProblem &, Command &);

        swProcessStatus getCommandActive(threadData *, qpProblem &, Command &);
        swProcessStatus getCommandWait(threadData *, qpProblem &, Command &);

        swProcessStatus getCommandFinalize(threadData *, qpProblem &, Command &);
        swProcessStatus getCommandFail(const swParameters &, qpProblem &, Command &);

        void computeStopCommand(Command &);
        qpReturn computeQPCommand(const swParameters&, qpProblem &, Command &);
};
