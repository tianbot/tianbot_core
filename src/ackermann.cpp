#include "ackermann.h"
#include "protocol.h"

void TianbotAckermann::ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    vector<uint8_t> buf;
    struct ackermann_cmd ackermann_cmd;
    uint8_t *out = (uint8_t *)&ackermann_cmd;

    ackermann_cmd.steering_angle = msg->steering_angle;
    ackermann_cmd.speed = msg->speed;

    buildCmd(buf, PACK_TYPE_ACKMAN_VEL, (uint8_t *)&ackermann_cmd, sizeof(ackermann_cmd));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
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

TianbotAckermann::TianbotAckermann(ros::NodeHandle *nh) : TianbotChasis(nh)
{
    ackermann_sub_ = nh_.subscribe("ackermann_cmd", 5, &TianbotAckermann::ackermannCallback, this);
    initDone_ = true;
}
