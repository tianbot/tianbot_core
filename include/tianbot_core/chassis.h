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
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "tianbot_core/SignalLight.h"
#include "tianbot_core/HaitaiCtrl.h"
#include "core.h"

#define DEFAULT_BASE_FRAME "base_link"
#define DEFAULT_ODOM_FRAME "odom"
#define DEFAULT_IMU_FRAME "imu_link"

#define DEFAULT_PUBLISH_TF true

using namespace std;
using namespace boost;

class TianbotChasis : public TianbotCore {
public:
    TianbotChasis(ros::NodeHandle *nh);

private:
    ros::Publisher odom_pub_;
    ros::Publisher uwb_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher voltage_pub_;
    ros::Publisher signal_light_pub_;  // 信号灯状态发布者
    ros::Publisher actuator_pub_;  // 推杆状态发布者
    ros::Publisher haitai_vel_pub_;  // 海泰电机速度发布者
    ros::Subscriber signal_light_sub_;    // 信号灯控制订阅者
    ros::Subscriber actuator_sub_;  // 推杆控制订阅者
    ros::Subscriber haitai_ctrl_sub_;  // 海泰电机控制订阅者
    ros::Subscriber cmd_vel_sub_;
    void signalLightCallback(const tianbot_core::SignalLight::ConstPtr &msg);  // 信号灯控制回调函数
    void actuatorCallback(const std_msgs::UInt8::ConstPtr &msg);  // 推杆控制回调函数 
    void haitaiCtrlCallback(const tianbot_core::HaitaiCtrl::ConstPtr &msg);

    geometry_msgs::TransformStamped odom_tf_;
    tf::TransformBroadcaster tf_broadcaster_;
    bool publish_tf_;
    std::string base_frame_;
    std::string odom_frame_;
    std::string imu_frame_;
    bool publisher_init_done;
    virtual void tianbotDataProc(unsigned char *buf, int len);
};

#endif
