#ifndef TIANBOT_AMP_H
#define TIANBOT_AMP_H

#include "ros/ros.h"
#include "chassis.h"
#include "geometry_msgs/Twist.h"

class TianbotAmp : public TianbotChasis
{
public:
    TianbotAmp(ros::NodeHandle *nh);
private:
    ros::Subscriber stack_light_sub_;    
    ros::Subscriber lift_actuator_sub_;  
    ros::Subscriber spindle_sub_;  
    ros::Subscriber cmd_vel_sub_;
    void stacklightCallback(const tianbot_core::SignalLight::ConstPtr &msg);
    void liftactuatorCallback(const std_msgs::UInt8::ConstPtr &msg);
    void spindleCallback(const tianbot_core::HaitaiCtrl::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif // TIANBOT_AMP_H