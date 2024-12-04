#include "core.h"
#include <rclcpp/rclcpp.hpp>

#include "ackermann.h"
#include "differential.h"
#include "omni.h"
#include "rover.h"
#include "chassis.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tianbot_core");

    std::string type;
    bool type_verify;

    TianbotCore *core = nullptr;

    node->declare_parameter("type",  rclcpp::PARAMETER_STRING);
    node->declare_parameter("type_verify", rclcpp::PARAMETER_BOOL);

    if (!node->get_parameter("type", type)) {
        type = DEFAULT_TYPE;
    }
    if (!node->get_parameter("type_verify", type_verify)) {
        type_verify = DEFAULT_TYPE_VERIFY;
    }

    if (type == "omni")
    {
        core = new TianbotOmni(node);
    }
    else if (type == "ackermann")
    {
        core = new TianbotAckermann(node);
        printf( "new TianbotAckermann(node) success/n");
    }
    else if (type == "diff")
    {
        core = new TianbotDifferential(node);
    }
    else if (type == "rover")
    {
        core = new TianbotRover(node);
    }
    else if (type == "arm")
    {
        // Add arm initialization here
    }

    if (type_verify)
    {
        core->checkDevType();
    }

    // rclcpp::Rate loop_rate(10);
    // while (rclcpp::ok())
    // {
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    // }
    rclcpp::spin(node);  //
    delete core;
    rclcpp::shutdown();
    return 0;
}
