#ifndef __ROVER_H__
#define __ROVER_H__

#include "ackermann.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define MOVE_TYPE_ACKERMAN 0
#define MOVE_TYPE_ROTATE 1
#define MOVE_TYPE_OMNI 2

class TianbotRover : public TianbotAckermann
{
  public:
    TianbotRover(ros::NodeHandle *nh);

  private:
    ros::Subscriber rover_sub_;
    void roverCallback(const std_msgs::String::ConstPtr &msg);
};

#endif
