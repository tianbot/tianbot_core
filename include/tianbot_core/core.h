#ifndef __CORE_H__
#define __CORE_H__

#include "ros/ros.h"
#include "serial.h"
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "string.h"

#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"

using namespace std;
using namespace boost;

class TianbotCore {
public:
    Serial serial_;
    string type;

    TianbotCore(ros::NodeHandle *nh);

    static string getType(void);

    virtual void tianbotDataProc(unsigned char *buf, int len) = 0;

private:
    ros::Timer heartbeat_timer_;
    ros::Timer communication_timer_;
    ros::NodeHandle nh_;

    void serialDataProc(uint8_t *data, unsigned int data_len);
    void heartCallback(const ros::TimerEvent &);
    void communicationErrorCallback(const ros::TimerEvent &);
};

#endif
