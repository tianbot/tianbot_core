#ifndef __CORE_H__
#define __CORE_H__

#include "ros/ros.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "string.h"
#include "std_msgs/String.h"
#include "tianbot_core/DebugCmd.h"
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
    ros::Publisher debug_result_pub_;
    ros::Subscriber debug_cmd_sub_;
    ros::NodeHandle nh_;
    ros::Timer heartbeat_timer_;
    ros::Timer communication_timer_;

    bool debugResultFlag_;
    string debugResultStr_;
    bool initDone_;

    TianbotCore(ros::NodeHandle *nh);
    void checkDevType(void);
    virtual void tianbotDataProc(unsigned char *buf, int len) = 0;
    void open(void);

private:

    void dataProc(uint8_t *data, unsigned int data_len);
    void heartCallback(const ros::TimerEvent &);
    void communicationErrorCallback(const ros::TimerEvent &);
    void debugCmdCallback(const std_msgs::String::ConstPtr &msg);
    bool debugCmdSrv(tianbot_core::DebugCmd::Request &req,  tianbot_core::DebugCmd::Response &res);

    ros::ServiceServer param_set_;
};

#endif
