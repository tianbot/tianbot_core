#include "omni.h"
#include "protocol.h"

void TianbotOmni::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{
    uint16_t len;
    std::vector<uint8_t> buf;

    twist twist;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, (uint8_t *)&twist, sizeof(twist));
        if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        RCLCPP_ERROR(this->node->get_logger(), "communication failed, reopen the device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();

}


TianbotOmni::TianbotOmni(const std::shared_ptr<rclcpp::Node> &node) : TianbotChasis(node)
{

    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&TianbotOmni::velocityCallback, this, std::placeholders::_1));
    initDone_ = true;

}
