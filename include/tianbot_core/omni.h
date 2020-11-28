#ifndef __OMNI_H__
#define __OMNI_H__

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

#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"
#define DEFAULT_BASE_FRAME "base_link"
#define DEFAULT_ODOM_FRAME "odom"

using namespace std;
using namespace boost;

class TianbotCore {
public:
    TianbotCore(ros::NodeHandle *nh);
    //~Tianboard();
private:
    ros::Publisher odom_pub_;
    ros::Publisher uwb_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::TransformStamped odom_tf_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer heartbeat_timer_;
    ros::Timer communication_timer_;
    ros::NodeHandle nh_;
    Serial serial_;
    std::string base_frame_;
    std::string odom_frame_;
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void serialDataProc(uint8_t *data, unsigned int data_len);
    void tianboardDataProc(unsigned char *buf, int len);
    void heartCallback(const ros::TimerEvent&);
    void communicationErrorCallback(const ros::TimerEvent&);
};

#endif


#endif