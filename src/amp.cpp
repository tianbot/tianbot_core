#include "amp.h"
#include "protocol.h"

TianbotAmp::TianbotAmp(ros::NodeHandle *nh) : TianbotChasis(nh)
{
    stack_light_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("stack_light_state", 1);  // 添加信号灯状态发布者
    lift_actuator_pub_ = nh_.advertise<std_msgs::UInt8>("lift_actuator_state", 1);  // 添加推杆状态发布者
    spindle_vel_pub_ = nh_.advertise<std_msgs::Float32>("spindle_state", 1);  // 添加海泰电机速度发布者
    stack_light_sub_ = nh_.subscribe<tianbot_core::SignalLight>("stack_light_ctrl", 1, &TianbotAmp::stacklightCallback, this);  
    lift_actuator_sub_ = nh_.subscribe<std_msgs::UInt8>("lift_actuator_ctrl", 1, &TianbotAmp::liftactuatorCallback, this);
    spindle_sub_ = nh_.subscribe<tianbot_core::HaitaiCtrl>("spindle_ctrl", 1, &TianbotAmp::spindleCallback, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &TianbotAmp::velocityCallback, this);
    
    initDone_ = true;
}

void TianbotAmp::stacklightCallback(const tianbot_core::SignalLight::ConstPtr &msg)
{
    struct signal_status signal_ctrl;
    signal_ctrl.red = msg->red;
    signal_ctrl.yellow = msg->yellow;
    signal_ctrl.green = msg->green;
    signal_ctrl.buzzer = msg->buzzer;

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_SIGNAL_CTRL, reinterpret_cast<uint8_t*>(&signal_ctrl), sizeof(signal_ctrl));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}
void TianbotAmp::liftactuatorCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    struct actuator_status actuator;
    actuator.state = msg->data;

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_ACTUATOR_CTRL, reinterpret_cast<uint8_t*>(&actuator), sizeof(actuator));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}
void TianbotAmp::spindleCallback(const tianbot_core::HaitaiCtrl::ConstPtr &msg)
{
    struct HaitaiCtrl_t haitai_ctrl;
    haitai_ctrl.position = msg->position;  // 目标位置 (rad)
    haitai_ctrl.velocity = msg->velocity;  // 目标速度 (rad/s)
    haitai_ctrl.torque = msg->torque;      // 直接力矩 (N·m)
    haitai_ctrl.kp = msg->kp;              // 位置增益
    haitai_ctrl.kd = msg->kd;              // 速度增益

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_HAITAI_CTRL, reinterpret_cast<uint8_t*>(&haitai_ctrl), sizeof(haitai_ctrl));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}

void TianbotAmp::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    uint16_t len;
    std::vector<uint8_t> buf;

    struct twist twist;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, (uint8_t *)&twist, sizeof(twist));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}