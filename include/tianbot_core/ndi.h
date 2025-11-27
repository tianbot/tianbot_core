#ifndef TIANBOT_NDI_H
#define TIANBOT_NDI_H

#include "ros/ros.h"
#include "chassis.h"
#include "geometry_msgs/Twist.h"
#include "tianbot_core/EmmV5Ctrl.h"
#include "std_msgs/UInt32.h"

class TianbotNdi : public TianbotChasis
{
public:
    TianbotNdi(ros::NodeHandle *nh);
    
private:
    ros::Subscriber stack_light_sub_;
    ros::Subscriber lift_actuator_sub_;
    ros::Subscriber line_opto_sub_;
    ros::Subscriber cmd_vel_sub_;
    void stacklightCallback(const tianbot_core::SignalLight::ConstPtr &msg);
    void liftActuatorCallback(const tianbot_core::EmmV5Ctrl::ConstPtr &msg);
    void lineOptoCallback(const std_msgs::UInt32::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif // TIANBOT_NDI_H
