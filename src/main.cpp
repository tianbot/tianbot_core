#include "ros/ros.h"
#include "core.h"
#include "chasis.h"
#include "string.h"

using namespace std;

int main(int argc, char *argv[])
{
    string type;
    uint32_t retry;
    ros::init(argc, argv, "tianbot_core");
    ros::NodeHandle nh("~");
    TianbotCore *core;

    retry = 5;
    while (--retry)
    {
        type = TianbotCore::getType();
        if (type == "omni")
        {
            core = new TianbotChasis(&nh);
        }
        else if (type == "ackermann")
        {

        }
        else if (type == "arm")
        {

        }
        else
        {
            ROS_ERROR("Unsupportted type: %s, retry %d...", type, retry);
        }
        ros::Duration(1).sleep();
    }
    if (retry == 0)
    {
        ROS_ERROR("Get product type error, please check hardware.");
        delete core;
        return -1;
    }

    ROS_INFO("Product type: %s", type);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete core;
    return 0;
}
