#include "ackermann.h"
#include "protocol.h"

void TianbotAckermann::ackermannCallback(const ackermann_msgs::msg::AckermannDrive::ConstPtr &msg)
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
        RCLCPP_ERROR(get_logger(), "communication failed, reopen the device");
        heartbeat_timer_.cancel();
        communication_timer_.cancel();
        open();
        communication_timer_.reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

TianbotAckermann::TianbotAckermann(const std::shared_ptr<rclcpp::Node> &node) : TianbotChasis(node)
{
    ackermann_sub_ = node->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "ackermann_cmd", 1, std::bind(&TianbotAckermann::ackermannCallback, this, std::placeholders::_1));

    initDone_ = true;
}
