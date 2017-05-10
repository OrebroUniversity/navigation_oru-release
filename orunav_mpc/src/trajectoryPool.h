#pragma once

#include <vector>
#include <map>


#include "commonDefines.h"
#include "state.h"
#include "control.h"
#include "sharedAccess.h"
#include "trajectory.h"
#include "trajectoryCache.h"


class trajectoryChunk
{
    public:
        /// The trajectory should not be updated if true.
        bool finalized;
        unsigned int sequence_num;

        /// Identificator
        trajectoryID            id;

        /// Steps
        deque<trajectoryStep>   steps;

        Constraints constraints;


        trajectoryChunk(const swParameters &);
        trajectoryChunk(const Constraints &);

        void finalize();
        unsigned int deleteFront(const unsigned int);
        unsigned int copy(const unsigned int, const unsigned int, trajectoryCache&) const;
};


class trajectoryPoolIndex
{
    public:
        unsigned int chunk;
        unsigned int step;

        void reset();
};


/**
 * @brief The evaluator requires some data to be in sync, the easiest way to achieve 
 * this is to create copies at a particular moment (this is done by controller).
 */
class evaluatorState
{
    public:
        /// Index of step on the active trajectory, corresponding to the current state
        /// for the next execution of evaluator.
        trajectoryPoolIndex index;

        /// Current state for the next execution of evaluator.
        State current_state;

        /// Last computed control
        Control last_control;
};


/**
 * @brief A pool of trajectories.
 */
class trajectoryPool
{
    public:
        trajectoryPool();

        void prune();
        void clear();

        void addTrajectoryChunk(const trajectoryChunk &);

        void setActive(const trajectoryID);
        
        void getActiveTrajectory(
                const State &, 
                const Control &, 
                trajectoryCache &);
        void getAllTrajectories(
                vector<trajectoryCache> &, 
                State &, 
                Control &);
	void getActiveTrajectoryIndexes(trajectoryPoolIndex &);

        void block ();
        void unblock ();
        bool isBlocked ();


    private:
        /// A mutex for shared access
        swMutex mutex;

        /// Trajectories
        map< trajectoryID, deque<trajectoryChunk> > traj_pool;

        /// Index of step on the active trajectory, corresponding to the current state.
        trajectoryPoolIndex index;

        /// State of the evaluator.
        evaluatorState eval_state;

        /// ID of the active trajectory.
        trajectoryID active_traj_id;

        /// ID of the active trajectory is set.
        bool active_id_is_set;

        /// The pool is blocked, this variable is used to enable tracking at a certain time.
        bool blocked;


        void copySteps(
                const deque<trajectoryChunk> &, 
                const trajectoryPoolIndex &,
                trajectoryCache &);
};
