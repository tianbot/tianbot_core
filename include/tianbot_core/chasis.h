#ifndef __CHASIS_H__
#define __CHASIS_H__

#include "ros/ros.h"
#include "serial.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "core.h"

#define DEFAULT_BASE_FRAME "base_link"
#define DEFAULT_ODOM_FRAME "odom"

using namespace std;
using namespace boost;

class TianbotChasis : public TianbotCore {
public:
    TianbotChasis(ros::NodeHandle *nh);

private:
    ros::Publisher odom_pub_;
    ros::Publisher uwb_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::TransformStamped odom_tf_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string base_frame_;
    std::string odom_frame_;
    virtual void tianbotDataProc(unsigned char *buf, int len);
};

#endif
