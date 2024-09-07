#include "core.h"
#include "ros/ros.h"

#include "ackermann.h"
#include "differential.h"
#include "omni.h"
#include "rover.h"
#include "stdint.h"
#include "string.h"

using namespace std;

int main(int argc, char *argv[])
{
    string type;
    bool type_verify;

    TianbotCore *core;

    ros::init(argc, argv, "tianbot_core");
    ros::NodeHandle nh("~");

    nh.param<std::string>("type", type, DEFAULT_TYPE);
    nh.param<bool>("type_verify", type_verify, DEFAULT_TYPE_VERIFY);

    if (type == "omni")
    {
        core = new TianbotOmni(&nh);
    }
    else if (type == "ackermann")
    {
        core = new TianbotAckermann(&nh);
    }
    else if (type == "diff")
    {
        core = new TianbotDifferential(&nh);
    }
    else if (type == "rover")
    {
        core = new TianbotRover(&nh);
    }
    else if (type == "arm")
    {
    }
    if (type_verify)
    {
        core->checkDevType();
    }
        
    ros::spin();

    delete core;
    return 0;
}
