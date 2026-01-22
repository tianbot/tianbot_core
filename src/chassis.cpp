#include "chassis.h"
#include "protocol.h"

void TianbotChasis::tianbotDataProc(unsigned char *buf, int len)
{
    if (!publisher_init_done)
    {
        return;
    }
    struct protocol_pack *p = (struct protocol_pack *)buf;
    switch (p->pack_type)
    {
    case PACK_TYPE_ODOM_RESPONSE:
        if (sizeof(struct odom) == p->len - 2)
        {
            nav_msgs::msg::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            rclcpp::Time current_time = clock_->now();
            odom_msg.header.stamp = current_time;
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            geometry_msgs::msg::Quaternion q = createQuaternionMsgFromYaw(pOdom->pose.yaw);          
            odom_msg.pose.pose.orientation = q;
            // set the velocity
            odom_msg.child_frame_id = base_frame_;
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            // publish the message
            odom_pub_->publish(odom_msg);
            if (publish_tf_)
            {
                odom_tf_.header.stamp = current_time;
                odom_tf_.header.frame_id = odom_frame_;
                odom_tf_.child_frame_id = base_frame_;
                odom_tf_.transform.translation.x = pOdom->pose.point.x;
                odom_tf_.transform.translation.y = pOdom->pose.point.y;
                odom_tf_.transform.translation.z = pOdom->pose.point.z;

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(odom_tf_);
            }
        }
        break;

    case PACK_TYPE_ODOM_V2_RESPONSE:
        if (sizeof(struct odom_v2) == p->len - 2)
        {
            nav_msgs::msg::Odometry odom_msg;
            struct odom_v2 *pOdom = (struct odom_v2 *)(p->data);
            rclcpp::Time current_time = clock_->now();
            odom_msg.header.stamp = current_time;
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            //vector3 x->roll y->pitch z->yaw
            geometry_msgs::msg::Quaternion q = createQuaternionMsgFromRollPitchYaw(pOdom->pose.rpy.x, pOdom->pose.rpy.y, pOdom->pose.rpy.z);    
            odom_msg.pose.pose.orientation = q;
            // set the velocity
            odom_msg.child_frame_id = base_frame_;
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            // publish the message
            odom_pub_->publish(odom_msg);
            if (publish_tf_)
            {
                odom_tf_.header.stamp = current_time;
                odom_tf_.header.frame_id = odom_frame_;
                odom_tf_.child_frame_id = base_frame_;
                odom_tf_.transform.translation.x = pOdom->pose.point.x;
                odom_tf_.transform.translation.y = pOdom->pose.point.y;
                odom_tf_.transform.translation.z = pOdom->pose.point.z;

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(odom_tf_);
            }
        }
        break;

    case PACK_TYPE_UWB_RESPONSE:
        if (sizeof(struct uwb) == p->len - 2)
        {
            geometry_msgs::msg::Pose2D pose2d_msg;
            auto pUwb = reinterpret_cast<struct uwb *>(p->data);
            pose2d_msg.x = pUwb->x_m;
            pose2d_msg.y = pUwb->y_m;
            pose2d_msg.theta = pUwb->yaw;
            uwb_pub_->publish(pose2d_msg);
        }
        break;

    case PACK_TYPE_Voltage_RESPONSE:
        if (sizeof(struct voltage) == p->len - 2)
        {
            std_msgs::msg::Float32 battery_msg;
            auto voltage = reinterpret_cast<struct voltage *>(p->data);
            battery_msg.data = voltage->Battery_voltage;
            voltage_pub_->publish(battery_msg);
        }
        break;

    case PACK_TYPE_HEART_BEAT_RESPONSE:
        break;

    case PACK_TYPE_IMU_REPONSE:
        if (sizeof(struct imu_feedback) == p->len - 2)
        {
            sensor_msgs::msg::Imu imu_msg;
            auto pImu = reinterpret_cast<struct imu_feedback *>(p->data);

            imu_msg.header.stamp = clock_->now();
            // imu_msg.header.frame_id = (nh_.getNamespace() + "/" + imu_frame_).erase(0,1);
            imu_msg.header.frame_id = imu_frame_;
            imu_msg.orientation.x = pImu->quat.x;
            imu_msg.orientation.y = pImu->quat.y;
            imu_msg.orientation.z = pImu->quat.z;
            imu_msg.orientation.w = pImu->quat.w;
            imu_msg.angular_velocity.x = pImu->angular_vel.x;
            imu_msg.angular_velocity.y = pImu->angular_vel.y;
            imu_msg.angular_velocity.z = pImu->angular_vel.z;
            imu_msg.linear_acceleration.x = pImu->linear_acc.x;
            imu_msg.linear_acceleration.y = pImu->linear_acc.y;
            imu_msg.linear_acceleration.z = pImu->linear_acc.z;
            imu_pub_->publish(imu_msg);
        }
        break;

    case PACK_TYPE_DEBUG_RESPONSE: {
            std_msgs::msg::String debug_msg;
            p->data[p->len - 2] = '\0';
            debug_msg.data = (char *)(p->data);
            debugResultStr_ = (char *)(p->data);
            debugResultFlag_ = true;
            debug_result_pub_->publish(debug_msg);
        }
        break;

    default:
        break;
    }
}

TianbotChasis::TianbotChasis(const std::shared_ptr<rclcpp::Node> & node)
    : TianbotCore(node), publisher_init_done(false)
{
    if (!node->get_parameter("base_frame", base_frame_)) {
        base_frame_ = DEFAULT_BASE_FRAME;
    }
    if (!node->get_parameter("odom_frame", odom_frame_)) {
        odom_frame_ = DEFAULT_ODOM_FRAME;
    }
    if (!node->get_parameter("imu_frame", imu_frame_)) {
        imu_frame_ = DEFAULT_IMU_FRAME;
    }
    if (!node->get_parameter("publish_tf", publish_tf_)) {
        publish_tf_ = DEFAULT_PUBLISH_TF;
    }
 
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    uwb_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("uwb", 1);
    voltage_pub_ = node->create_publisher<std_msgs::msg::Float32>("voltage", 1);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    publisher_init_done = true;

    odom_tf_.header.frame_id = odom_frame_;
    odom_tf_.child_frame_id = base_frame_;
}
