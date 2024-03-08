#ifndef __ROVER_H__
#define __ROVER_H__

#include "ackermann.h"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#define MOVE_TYPE_ACKERMAN 0
#define MOVE_TYPE_ROTATE 1
#define MOVE_TYPE_OMNI 2

class TianbotRover : public TianbotAckermann
{
public:
    TianbotRover(const std::shared_ptr<rclcpp::Node> &node);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rover_sub_;
    void roverCallback(const std_msgs::msg::String::ConstPtr &msg);
};

#endif
