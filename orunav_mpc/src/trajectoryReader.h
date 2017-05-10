#pragma once

#include "commonDefines.h"
#include "headers_ros.h"
#include "threadData.h"
#include "trajectory.h"


/******************************************/
/**
 * @brief A simple reader of trajectories from file.
 */
class trajectoryReader
{
    public:
        threadReturn readLoop(threadData *);

	void processTrajectoryChunkVec(const orunav_msgs::ControllerTrajectoryChunkVecConstPtr&);
        void processTrajectoryChunk(const orunav_msgs::ControllerTrajectoryChunkConstPtr&);
        void processTrajectoryChunk_(const orunav_msgs::ControllerTrajectoryChunk&);
        void actOnCommand(const orunav_msgs::ControllerCommandConstPtr&);
        void checkActivity(const orunav_msgs::ActiveRobotsConstPtr&);


    private:
        /// A local pointer to the shared data
        threadData *thdata;

        /// When to start tracking.
        ros::Time start_time;

        /// Activity flag.
        bool is_active;
        
        void initStepConstraints(orunav_msgs::ControllerTrajectoryChunk::ConstPtr, Constraints &);
	void initStepConstraints_(const orunav_msgs::ControllerTrajectoryChunk&, Constraints &);

};
