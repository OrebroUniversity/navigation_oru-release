#include "ROSSensorSubscriber.h"

#include <boost/bind.hpp>

#include "swLogger.h"


/**
 * @brief The reader loop, which runs within a separate thread.
 *
 * @param[in] thread_data thread data.
 */
threadReturn ROSSensorSubscriber::readLoop(threadData *thread_data)
{
    robot_id = thread_data->parameters.robot_id;
    threadReturn retval = SW_THREAD_RETURN_DONE;

    ros::CallbackQueue callback_queue;
    ros::Subscriber ros_subscriber;


    ros::SubscribeOptions sub_opts = ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
            thread_data->parameters.topic_simulation_sensor, 
            100,
            boost::bind(&ROSSensorSubscriber::sensorCallback, this, _1),
            ros::VoidPtr(),
            &callback_queue);


    ros_subscriber = thread_data->ros_node.subscribe(sub_opts);


    for(unsigned int counter = 0 ;; )
    {
        if (thread_data->isProcessTerminated())
        {
            retval = SW_THREAD_RETURN_STOPPED;
            break;
        }

        callback_queue.callOne (ros::WallDuration(SW_SIM_CALLBACK_WAIT_SEC));
        ++counter;
        if (counter == SW_SIM_UPDATES_PER_SAMPLING_PERIOD)
        {
            counter = 0;

            ros::Time now = ros::Time::now();
            // Wait until everyting is initialized.
            if (now.sec == 0) 
            {
                continue;
            }

            SW_LOG_SENSOR("[" << now << "]  " << current_state);

            thread_data->state.set(thread_data->parameters, current_state);
            thread_data->state.signal();
        }
    }

    return (retval);
}



/**
 * @brief This callback is used to process messages.
 *
 * @param[in] msg_link_st link state message.
 */
void ROSSensorSubscriber::sensorCallback (const gazebo_msgs::LinkStates::ConstPtr & msg_link_st)
{
    // Parse the messages
    std::ostringstream o_pose, o_phi;
    o_pose << "robot" << robot_id << "::base_footprint";
    o_phi << "robot" << robot_id << "::steer_link"; 
    for (unsigned int i = 0; i < msg_link_st->name.size (); ++i)
    {
        if (msg_link_st->name[i] == o_pose.str()) // "snowwhite::base_footprint")
        {
	    current_state.x() = msg_link_st->pose[i].position.x;
            current_state.y() = msg_link_st->pose[i].position.y;
            current_state.theta() = tf::getYaw(msg_link_st->pose[i].orientation);
	}
        if (msg_link_st->name[i] == o_phi.str()) //"snowwhite::steer_link")
        {
            current_state.phi() = tf::getYaw(msg_link_st->pose[i].orientation);
        }
    }
    // convert phi from the global reference frame
    current_state.phi() = current_state.phi() - current_state.theta();
    current_state.normalizePhi();

    //    std::cerr << "current_state [in] : " << current_state << std::endl;
}
