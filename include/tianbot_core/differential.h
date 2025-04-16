#ifndef __DIFFERENTIAL_H__
#define __DIFFERENTIAL_H__

#include <rclcpp/rclcpp.hpp>
#include "chassis.h"
#include "geometry_msgs/msg/twist.hpp"

class TianbotDifferential : public TianbotChasis {
public:
    TianbotDifferential(const std::shared_ptr<rclcpp::Node> &node);
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
};

#endif
