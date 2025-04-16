#ifndef __CORE_H__
#define __CORE_H__

#include "rclcpp/rclcpp.hpp"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "string.h"
#include "std_msgs/msg/string.hpp"
#include "tianbot_core/srv/debug_cmd.hpp"
#include <string>
#include "serial.h"
#include "udp.h"
#include "comm_if.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"

using namespace boost;

#define DEFAULT_SERIAL_BAUDRATE 460800
#define DEFAULT_CLIENT_PORT 8888
#define DEFAULT_SERVER_PORT 6666
#define DEFAULT_TYPE "omni"
#define DEFAULT_TYPE_VERIFY true

using namespace std;
using namespace boost;

class TianbotCore
{
public:
    CommInterface *comm_inf_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_result_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_cmd_sub_;
    rclcpp::Service<tianbot_core::srv::DebugCmd>::SharedPtr debug_cmd_srv_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr communication_timer_;
    rclcpp::Node::SharedPtr node;

    bool debugResultFlag_;
    string debugResultStr_;
    bool initDone_;

    TianbotCore(const std::shared_ptr<rclcpp::Node> &nh);
    void checkDevType(void);
    virtual void tianbotDataProc(unsigned char *buf, int len) = 0;
    void open(void);
    virtual ~TianbotCore() {};

private:

    void dataProc(uint8_t *data, unsigned int data_len);
    void heartCallback();
    void communicationErrorCallback();
    void debugCmdCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
    bool debugCmdSrv(const std::shared_ptr<tianbot_core::srv::DebugCmd::Request> req, 
                            std::shared_ptr<tianbot_core::srv::DebugCmd::Response> res);

    rclcpp::Service<tianbot_core::srv::DebugCmd>::SharedPtr param_set_;
};

#endif
