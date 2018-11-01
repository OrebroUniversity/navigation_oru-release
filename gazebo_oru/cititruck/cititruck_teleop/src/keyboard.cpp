/*
 * Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_O 0x6F
#define KEYCODE_P 0x70

class ErraticKeyboardTeleopNode
{
  private:
    double walk_vel_;
    double yaw_rate_;
      
    geometry_msgs::Twist cmdvel_;
    geometry_msgs::Point cmdfork_;
    
    ros::NodeHandle n_;
    
    ros::Publisher pub_vel_;
    ros::Publisher pub_fork_;

  public:
    ErraticKeyboardTeleopNode()
    {
      cmdfork_.x = 0;
      cmdfork_.y = 0;
      cmdfork_.z = 0;
    }
    
    ~ErraticKeyboardTeleopNode() { }
    
    void Init( int RobotIdentification)
    {
      char * Buffer = new char[100];
      sprintf( Buffer, "/robot%d/controller/cmd_vel", RobotIdentification);
      pub_vel_ = n_.advertise<geometry_msgs::Twist>( Buffer, 1);
      
      ros::NodeHandle n_private("~");
      n_private.param("walk_vel", walk_vel_, 0.1);
      n_private.param("yaw_rate", yaw_rate_, 1.0);
      
      //advertise another topic (controlling the forks)
      sprintf( Buffer, "/robot%d/controller/cmd_fork", RobotIdentification);
      pub_fork_ = n_.advertise<geometry_msgs::Point>( Buffer, 1);      
      
      delete [] Buffer;
    }
    
    void keyboardLoop();
    
    void stopRobot()
    {
      cmdvel_.linear.x = 0.0;
      cmdvel_.angular.z = 0.0;
      pub_vel_.publish(cmdvel_);
    }
};

ErraticKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;
int RobotID;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

  ErraticKeyboardTeleopNode tbk;
  
  if( argc > 1)
  {
    sscanf( argv[1], "%d", &RobotID);
    tbk.Init( RobotID);
  }
  else
  {
    RobotID = 1;
    tbk.Init( RobotID);
  }
  
  boost::thread t = boost::thread(boost::bind(&ErraticKeyboardTeleopNode::keyboardLoop, &tbk));
  
  ros::spin();
  
  t.interrupt();
  t.join();
  tbk.stopRobot();
  tcsetattr(kfd, TCSANOW, &cooked);
  
  return(0);
}

void ErraticKeyboardTeleopNode::keyboardLoop()
{
  char c;
  double max_tv = walk_vel_;
  double max_rv = yaw_rate_;
  bool dirty = false;
  int speed = 0;
  int turn = 0;
  char Buffer[100];
  
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  
  puts("Reading from keyboard");
  puts("Use ");
  puts("  WSAD keys to control the robot");
  puts("  PO   keys to lift and put down the forks");
  sprintf( Buffer, "Running keyboard control for robot %d", RobotID);
  puts( Buffer);
  
  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  
  for(;;)
  {
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    
    if ((num = poll(&ufd, 1, 250)) < 0)
    {
      perror("poll():");
      return;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return;
      }
    }
    else
    {
      if (dirty == true)
      {
        //        stopRobot();
        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = 0;
        pub_vel_.publish(cmdvel_);

        dirty = false;
      }
      
	    continue;
    }
    
    switch(c)
    {
      case KEYCODE_W:
        max_tv = walk_vel_;
        speed++;
        turn = 0;
        dirty = true;
        break;
      case KEYCODE_S:
        max_tv = walk_vel_;
        speed--;
        turn = 0;
        dirty = true;
        break;
      case KEYCODE_A:
        max_rv = yaw_rate_;
        //speed = 0;
        turn = 1;
        dirty = true;
        break;
      case KEYCODE_D:
        max_rv = yaw_rate_;
        //speed = 0;
        turn = -1;
        dirty = true;
        break;
      
      case KEYCODE_O:
      {
        cmdfork_.z = 0;
        break;
      }
      case KEYCODE_P:
      {
        cmdfork_.z = 0.1;
        break;
      }
        
        
      default:
      {
        max_tv = walk_vel_;
        max_rv = yaw_rate_;
        speed = 0;
        turn = 0;
        dirty = false;
        
        break;
      }
    }
    
    cmdvel_.linear.x = speed * max_tv;
    cmdvel_.angular.z = turn * max_rv;
    pub_vel_.publish(  cmdvel_ );
    pub_fork_.publish( cmdfork_);
  }
}

