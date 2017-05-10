#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <orunav_msgs/ForkCommand.h>
#include <orunav_msgs/ForkReport.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread/mutex.hpp>


class ForkControlSimNode {

    private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher forkreport_pub_;
    ros::Publisher forkcommand_pub_;
    ros::Subscriber forkcommand_sub_;
    ros::Timer heartbeat_reports_;
    bool visualize;
    double lift_threshold_;
    orunav_msgs::ForkReport current_report_;
    double fork_max_speed_;
    boost::mutex reports_mutex_;
public: 
	ForkControlSimNode (ros::NodeHandle &paramHandle) {
	  

	    forkcommand_sub_ = nh_.subscribe<orunav_msgs::ForkCommand>("fork/command",0,&ForkControlSimNode::process_forkcommand,this);
            double hb_report;
            paramHandle.param("heartbeat",hb_report,0.1);
	    paramHandle.param<bool>("visualize",visualize,true);
            paramHandle.param<double>("lift_threshold", lift_threshold_, 0.05);
            paramHandle.param<double>("fork_max_speed", fork_max_speed_, 0.04);

            forkreport_pub_ = nh_.advertise<orunav_msgs::ForkReport>("fork/report", 1000);            
            forkcommand_pub_ = nh_.advertise<geometry_msgs::Point>("cmd_fork", 1000);

  	    if (visualize)
	    {
                std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
                marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	    }
            
            
	    heartbeat_reports_   = nh_.createTimer(ros::Duration(hb_report),&ForkControlSimNode::publish_reports,this);

            current_report_.status = current_report_.FORK_POSITION_UNKNOWN;
            current_report_.state.position_z = 0.;
        }

	~ForkControlSimNode() {

        }
  
	void publish_reports(const ros::TimerEvent& event) {
//        ROS_INFO("[ForkControlSim]: Publishing reports.");
	  reports_mutex_.lock();
          current_report_.stamp = ros::Time::now();
	  forkreport_pub_.publish(current_report_);
	  reports_mutex_.unlock();
	}

    
    void send_interpolated_fork_cmds(double current, double target, int rate) {
        ros::Rate r(rate);
        double time_to_move_forks = fabs(target - current)/this->fork_max_speed_;
        int steps = time_to_move_forks * 10;
        double inc_z = (target - current) / (steps*1.);
        geometry_msgs::Point cmdfork; cmdfork.x = 0; cmdfork.y = 0;
        for (int i = 0; i <= steps; i++) {
            cmdfork.z = current + inc_z * i;
            ROS_INFO("[ForkControlSim]: sending new fork command z : %f", cmdfork.z);
            forkcommand_pub_.publish(cmdfork);
            r.sleep();
        }
    }


    void process_forkcommand(const orunav_msgs::ForkCommand::ConstPtr &msg) {

        // NOTE: This is written to be rather citi truck compatible and not to provide more information (than the cititruck) -> only to move up and down + we don't really know where the forks are.
        ROS_INFO("[ForkControlSim]: Got ForkCommand z : %f", msg->state.position_z);
        if (msg->state.position_z > lift_threshold_) {
            // Lift the forks (unless they are already up).
            ROS_INFO("[ForkControlSim]: Forks - high");
            if (current_report_.status != current_report_.FORK_POSITION_HIGH)
            {
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_MOVING_UP;
                double current_pos_z = current_report_.state.position_z;
                reports_mutex_.unlock();

                send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);
                
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_POSITION_HIGH;
                current_report_.state.position_z = msg->state.position_z; // Assume we're up.
                reports_mutex_.unlock();
            }
        }
        else if (msg->state.position_z < -0.01) {
          // Support legs...
          ROS_INFO("[ForkControlSim]: Forks - activate support legs");
          if (current_report_.status != current_report_.FORK_POSITION_SUPPORT_LEGS)
            {
              reports_mutex_.lock();
              current_report_.status = current_report_.FORK_MOVING_DOWN;
              double current_pos_z = current_report_.state.position_z;
              reports_mutex_.unlock();
              
              send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);
              
              reports_mutex_.lock();
              current_report_.status = current_report_.FORK_POSITION_SUPPORT_LEGS;
              current_report_.state.position_z = msg->state.position_z; // Assume we're down.
              reports_mutex_.unlock();
            }
        }
        else
        {
            // Lower the forks (unless they are already low).
            ROS_INFO("[ForkControlSim]: Forks - low");
            if (current_report_.status != current_report_.FORK_POSITION_LOW)
            {
                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_MOVING_DOWN;
                double current_pos_z = current_report_.state.position_z;
                reports_mutex_.unlock();

                send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);

                reports_mutex_.lock();
                current_report_.status = current_report_.FORK_POSITION_LOW;
                current_report_.state.position_z = msg->state.position_z; // Assume we're down.
                reports_mutex_.unlock();
            }
        }
    }
};


using namespace std;

int main(int argc, char** argv) {

    ros::init(argc,argv,"fork_control_sim");
    ros::NodeHandle params ("~");

    ForkControlSimNode node(params);

    ros::spin();
}
