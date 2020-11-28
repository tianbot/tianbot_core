#include "chasis.h"
#include "protocol.h"

void TianbotChasis::tianbotDataProc(unsigned char *buf, int len)
{
    struct protocol_pack *p = (struct protocol_pack *)buf;
    switch (p->pack_type)
    {
    case PACK_TYPE_ODOM_RESPONSE:
        if (sizeof(struct odom) == p->len - 2)
        {
            nav_msgs::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            ros::Time current_time = ros::Time::now();
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pOdom->pose.yaw);
            odom_msg.pose.pose.orientation = q;
            //set the velocity
            odom_msg.child_frame_id = base_frame_;
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            //publish the message
            odom_pub_.publish(odom_msg);

            odom_tf_.header.stamp = current_time;
            odom_tf_.transform.translation.x = pOdom->pose.point.x;
            odom_tf_.transform.translation.y = pOdom->pose.point.y;
            odom_tf_.transform.translation.z = pOdom->pose.point.z;

            odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
            tf_broadcaster_.sendTransform(odom_tf_);
        }
        break;

    case PACK_TYPE_UWB_RESPONSE:
        if (sizeof(struct uwb) == p->len - 2)
        {
            geometry_msgs::Pose2D pose2d_msg;
            struct uwb *pUwb = (struct uwb *)(p->data);
            pose2d_msg.x = pUwb->x_m;
            pose2d_msg.y = pUwb->y_m;
            pose2d_msg.theta = pUwb->yaw;
            uwb_pub_.publish(pose2d_msg);
        }
        break;

    case PACK_TYPE_HEART_BEAT_RESPONSE:
        break;

    case PACK_TYPE_IMU_REPONSE:
        if (sizeof(struct imu_feedback) == p->len - 2)
        {
            sensor_msgs::Imu imu_msg;
            struct imu_feedback *pImu = (struct imu_feedback *)(p->data);

            ros::Time current_time = ros::Time::now();
            imu_msg.header.stamp = current_time;
            imu_msg.header.frame_id = "imu";
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
            imu_pub_.publish(imu_msg);
        }
        break;

    default:
        break;
    }
}

void TianbotChasis::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    uint16_t len;
    vector<uint8_t> buf;

    uint8_t bcc = 0;
    struct twist twist;
    int i;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;

    buf.push_back(PROTOCOL_HEAD & 0xFF);
    buf.push_back((PROTOCOL_HEAD >> 8) & 0xFF);

    len = sizeof(struct twist) + 2;

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(PACK_TYPE_CMD_VEL & 0xFF);
    buf.push_back((PACK_TYPE_CMD_VEL >> 8) & 0xFF);

    for (i = 0; i < sizeof(struct twist); i++)
    {
        buf.push_back(out[i]);
    }

    for (i = 4; i < buf.size(); i++)
    {
        bcc ^= buf[i];
    }

    buf.push_back(bcc);

    serial_.send(&buf[0], buf.size());
    //heart_timer_.stop();
    //heart_timer_.start();
}

TianbotChasis::TianbotChasis(ros::NodeHandle *nh) : TianbotCore(nh)
{
    std::string param_serial_port;

    nh_.param<std::string>("base_frame", base_frame_, DEFAULT_BASE_FRAME);
    nh_.param<std::string>("odom_frame", odom_frame_, DEFAULT_ODOM_FRAME);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    uwb_pub_ = nh_.advertise<geometry_msgs::Pose2D>("uwb", 1);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &TianbotChasis::velocityCallback, this);
    odom_tf_.header.frame_id = odom_frame_;
    odom_tf_.child_frame_id = base_frame_;
}