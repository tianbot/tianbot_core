#ifndef __ACKERMANN_H__
#define __ACKERMANN_H__

#include <rclcpp/rclcpp.hpp>
#include "chassis.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

class TianbotAckermann : public TianbotChasis
{
public:
    TianbotAckermann(const std::shared_ptr<rclcpp::Node> &node);

private:
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;
    void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::ConstPtr &msg);
};

#endif
