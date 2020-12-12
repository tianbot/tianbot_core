#ifndef __CORE_H__
#define __CORE_H__

#include "ros/ros.h"
#include "serial.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "string.h"

#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"
#define DEFAULT_TYPE "mecanum"

using namespace std;
using namespace boost;

class TianbotCore
{
public:
    Serial serial_;
    string type;
    ros::NodeHandle nh_;
    ros::Timer heartbeat_timer_;

    TianbotCore(ros::NodeHandle *nh);

    virtual void tianbotDataProc(unsigned char *buf, int len) = 0;

private:

    ros::Timer communication_timer_;

    void serialDataProc(uint8_t *data, unsigned int data_len);
    void heartCallback(const ros::TimerEvent &);
    void communicationErrorCallback(const ros::TimerEvent &);
};

#endif
