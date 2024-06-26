#include "differential.h"
#include "protocol.h"

void TianbotDifferential::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    uint16_t len;
    vector<uint8_t> buf;

    struct twist twist;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, (uint8_t *)&twist, sizeof(twist));
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

TianbotDifferential::TianbotDifferential(ros::NodeHandle *nh) : TianbotChasis(nh)
{
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &TianbotDifferential::velocityCallback, this);
    initDone_ = true;
}
