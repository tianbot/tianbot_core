#include "rover.h"
#include "protocol.h"

void TianbotRover::roverCallback(const std_msgs::String::ConstPtr &msg)
{
    vector<uint8_t> buf;
    struct motion_mode motion_mode;
    uint8_t *out = (uint8_t *)&motion_mode;

    if (msg->data == "ackermann")
    {
        motion_mode.mode = MOVE_TYPE_ACKERMAN;
        ROS_INFO("rover motion mode set to ackermann");
    }
    else if (msg->data == "rotate")
    {
        motion_mode.mode = MOVE_TYPE_ROTATE;
        ROS_INFO("rover motion mode set to rotate");
    }
    else if (msg->data == "omni")
    {
        motion_mode.mode = MOVE_TYPE_OMNI;
        ROS_INFO("rover motion mode set to omni");
    }
    else
    {
        ROS_WARN("rover motion mode set failed, only ackermann / rotate / omni supported!");
    }

    buildCmd(buf, PACK_TYPE_SET_ROVER_MOTION_MODE, (uint8_t *)&motion_mode, sizeof(motion_mode));
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

TianbotRover::TianbotRover(ros::NodeHandle *nh)
    : TianbotAckermann(nh)
{
    rover_sub_ = nh_.subscribe("rover_motion_mode", 5, &TianbotRover::roverCallback, this);
}
