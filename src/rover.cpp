#include "rover.h"
#include "protocol.h"

void TianbotRover::roverCallback(const std_msgs::msg::String::ConstPtr &msg)
{
    vector<uint8_t> buf;
    struct motion_mode motion_mode;
    uint8_t *out = (uint8_t *)&motion_mode;

    if (msg->data == "ackermann")
    {
        motion_mode.mode = MOVE_TYPE_ACKERMAN;
        RCLCPP_INFO(get_logger(), "rover motion mode set to ackermann");
    }
    else if (msg->data == "rotate")
    {
        motion_mode.mode = MOVE_TYPE_ROTATE;
        RCLCPP_INFO(get_logger(), "rover motion mode set to rotate");
    }
    else if (msg->data == "omni")
    {
        motion_mode.mode = MOVE_TYPE_OMNI;
        RCLCPP_INFO(get_logger(), "rover motion mode set to omni");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "rover motion mode set failed, only ackermann / rotate / omni supported!");
    }

    buildCmd(buf, PACK_TYPE_SET_ROVER_MOTION_MODE, (uint8_t *)&motion_mode, sizeof(motion_mode));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(get_logger(), "communication failed, reopen the device");
        heartbeat_timer_.cancel();
        communication_timer_.cancel();
        open();
        communication_timer_.reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}


TianbotRover::TianbotRover(const std::shared_ptr<rclcpp::Node> &node) : TianbotAckermann(node)
{
    rover_sub_ = node->create_subscription<std_msgs::msg::String>(
        "rover_motion_mode", 1, std::bind(&TianbotRover::roverCallback, this, std::placeholders::_1));

    initDone_ = true;
}
