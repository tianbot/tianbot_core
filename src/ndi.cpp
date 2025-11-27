#include "ndi.h"
#include "protocol.h"

TianbotNdi::TianbotNdi(ros::NodeHandle *nh) : TianbotChasis(nh)
{
    stack_light_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("stack_light_state", 1);
    lift_actuator_pub_ = nh_.advertise<std_msgs::Float32>("lift_actuator_state", 1);
    stack_light_sub_ = nh_.subscribe<tianbot_core::SignalLight>("stack_light_ctrl", 1, &TianbotNdi::stacklightCallback, this);
    lift_actuator_sub_ = nh_.subscribe<tianbot_core::EmmV5Ctrl>("lift_actuator_ctrl", 1, &TianbotNdi::liftActuatorCallback, this);
    // 线光耦控制订阅：接收脉冲时长（ms），下发到下位机的 PACK_TYPE_LINE_OPTO_CTRL
    line_opto_sub_ = nh_.subscribe<std_msgs::UInt32>("line_opto_ctrl", 1, &TianbotNdi::lineOptoCallback, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &TianbotNdi::velocityCallback, this);

    initDone_ = true;
}

void TianbotNdi::stacklightCallback(const tianbot_core::SignalLight::ConstPtr &msg)
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

void TianbotNdi::liftActuatorCallback(const tianbot_core::EmmV5Ctrl::ConstPtr &msg)
{
    // 将 ROS 消息转换为协议中定义的 EmmV5 控制结构体
    struct EmmV5Ctrl_t emm_ctrl;
    emm_ctrl.distance_mm = msg->distance_mm; // 距离（mm）
    emm_ctrl.vel = msg->vel;                 // 速度（uint16）
    emm_ctrl.acc = msg->acc;                 // 加速度（uint8）

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_EMM_V5_CTRL, reinterpret_cast<uint8_t*>(&emm_ctrl), sizeof(emm_ctrl));
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

void TianbotNdi::lineOptoCallback(const std_msgs::UInt32::ConstPtr &msg)
{
    struct line_opto_cmd cmd;
    cmd.pulse_ms = msg->data; // 单位：毫秒

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_LINE_OPTO_CTRL, reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed when sending LINE_OPTO_CTRL, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}
void TianbotNdi::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
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