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
            nav_msgs::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            ros::Time current_time = ros::Time::now();
            odom_msg.header.stamp = current_time;
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pOdom->pose.yaw);
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
            odom_pub_.publish(odom_msg);
            if (publish_tf_)
            {
                odom_tf_.header.stamp = current_time;
                odom_tf_.transform.translation.x = pOdom->pose.point.x;
                odom_tf_.transform.translation.y = pOdom->pose.point.y;
                odom_tf_.transform.translation.z = pOdom->pose.point.z;

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_.sendTransform(odom_tf_);
            }
        }
        break;

    case PACK_TYPE_ODOM_V2_RESPONSE:
        if (sizeof(struct odom_v2) == p->len - 2)
        {
            nav_msgs::Odometry odom_msg;
            struct odom_v2 *pOdom = (struct odom_v2 *)(p->data);
            ros::Time current_time = ros::Time::now();
            odom_msg.header.stamp = current_time;
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            //vector3 x->roll y->pitch z->yaw
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(pOdom->pose.rpy.x, pOdom->pose.rpy.y, pOdom->pose.rpy.z);
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
            odom_pub_.publish(odom_msg);
            if (publish_tf_)
            {
                odom_tf_.header.stamp = current_time;
                odom_tf_.transform.translation.x = pOdom->pose.point.x;
                odom_tf_.transform.translation.y = pOdom->pose.point.y;
                odom_tf_.transform.translation.z = pOdom->pose.point.z;

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_.sendTransform(odom_tf_);
            }
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

    case PACK_TYPE_Voltage_RESPONSE:
        if (sizeof(struct voltage) == p->len - 2)
        {
            std_msgs::Float32 battery_msg;
            struct voltage *pvoltage = (struct voltage *)(p->data);
            battery_msg.data = pvoltage->Battery_voltage;
            voltage_pub_.publish(battery_msg);
        }
        break;
    
    case PACK_TYPE_SIGNAL_STATUS:
        if (sizeof(struct signal_status) == p->len - 2)
        {
            std_msgs::UInt8MultiArray signal_msg;
            struct signal_status *pSignal = (struct signal_status *)(p->data);
            signal_msg.data.push_back(pSignal->red);
            signal_msg.data.push_back(pSignal->yellow);
            signal_msg.data.push_back(pSignal->green);
            signal_msg.data.push_back(pSignal->buzzer);
            signal_light_pub_.publish(signal_msg);
        }
        break;
    
    case PACK_TYPE_ACTUATOR_STATUS:
        if (sizeof(struct actuator_status) == p->len - 2)
        {
            std_msgs::UInt8 actuator_msg;
            struct actuator_status *pActuator = (struct actuator_status *)(p->data);
            actuator_msg.data = pActuator->state;
            actuator_pub_.publish(actuator_msg);
        }
        break;
    case PACK_TYPE_HAITAI_VELOCITY:
        if (sizeof(struct haitai_vel) == p->len - 2)
        {
            std_msgs::Float32 haitai_vel_msg;
            struct haitai_vel *pHaitaiVel = (struct haitai_vel *)(p->data);
            haitai_vel_msg.data = pHaitaiVel->velocity;
            haitai_vel_pub_.publish(haitai_vel_msg);
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
            imu_pub_.publish(imu_msg);
        }
        break;

    case PACK_TYPE_DEBUG_RESPONSE: {
        std_msgs::String debug_msg;
        p->data[p->len - 2] = '\0';
        debug_msg.data = (char *)(p->data);
        debugResultStr_ = (char *)(p->data);
        debugResultFlag_ = true;
        debug_result_pub_.publish(debug_msg);
    }
    break;

    default:
        break;
    }
}
void TianbotChasis::signalLightCallback(const tianbot_core::SignalLight::ConstPtr &msg)
{
    struct signal_status signal_ctrl;
    signal_ctrl.red = msg->red;
    signal_ctrl.yellow = msg->yellow;
    signal_ctrl.green = msg->green;
    signal_ctrl.buzzer = msg->buzzer;

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_SIGNAL_CTRL, reinterpret_cast<uint8_t*>(&signal_ctrl), sizeof(signal_ctrl));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}
void TianbotChasis::actuatorCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    struct actuator_status actuator;
    actuator.state = msg->data;

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_ACTUATOR_CTRL, reinterpret_cast<uint8_t*>(&actuator), sizeof(actuator));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}
void TianbotChasis::haitaiCtrlCallback(const tianbot_core::HaitaiCtrl::ConstPtr &msg)
{
    struct HaitaiCtrl_t haitai_ctrl;
    haitai_ctrl.position = msg->position;  // 目标位置 (rad)
    haitai_ctrl.velocity = msg->velocity;  // 目标速度 (rad/s)
    haitai_ctrl.torque = msg->torque;      // 直接力矩 (N·m)
    haitai_ctrl.kp = msg->kp;              // 位置增益
    haitai_ctrl.kd = msg->kd;              // 速度增益

    std::vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_HAITAI_CTRL, reinterpret_cast<uint8_t*>(&haitai_ctrl), sizeof(haitai_ctrl));
    if (comm_inf_ && comm_inf_->send(&buf[0], buf.size()) != 0) {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.stop();
    heartbeat_timer_.start();
}

TianbotChasis::TianbotChasis(ros::NodeHandle *nh)
    : TianbotCore(nh), publisher_init_done(false)
{
    nh_.param<std::string>("base_frame", base_frame_, DEFAULT_BASE_FRAME);
    nh_.param<std::string>("odom_frame", odom_frame_, DEFAULT_ODOM_FRAME);
    nh_.param<std::string>("imu_frame", imu_frame_, DEFAULT_IMU_FRAME);

    nh_.param<bool>("publish_tf", publish_tf_, DEFAULT_PUBLISH_TF);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
    uwb_pub_ = nh_.advertise<geometry_msgs::Pose2D>("uwb", 1);
    voltage_pub_ = nh_.advertise<std_msgs::Float32>("voltage", 1);
    signal_light_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("signal_light", 1);  // 添加信号灯状态发布者
    actuator_pub_ = nh_.advertise<std_msgs::UInt8>("actuator", 1);  // 添加推杆状态发布者
    haitai_vel_pub_ = nh_.advertise<std_msgs::Float32>("haitai_vel", 1);  // 添加海泰电机速度发布者
    signal_light_sub_ = nh_.subscribe<tianbot_core::SignalLight>("signal_light_ctrl", 1, &TianbotChasis::signalLightCallback, this);  
    actuator_sub_ = nh_.subscribe<std_msgs::UInt8>("actuator_ctrl", 1, &TianbotChasis::actuatorCallback, this);
    haitai_ctrl_sub_ = nh_.subscribe<tianbot_core::HaitaiCtrl>("haitai_ctrl", 1, &TianbotChasis::haitaiCtrlCallback, this);
    publisher_init_done = true;

    odom_tf_.header.frame_id = odom_frame_;
    odom_tf_.child_frame_id = base_frame_;
}
