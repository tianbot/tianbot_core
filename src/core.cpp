#include "core.h"
#include "protocol.h"
#include <stdint.h>
#include <vector>

void TianbotCore::dataProc(uint8_t *data, unsigned int data_len)
{
    static uint8_t state = 0;
    uint8_t *p = data;
    static vector<uint8_t> recv_msg;
    static uint32_t len;
    uint32_t j;

    while (data_len != 0)
    {
        switch (state)
        {
        case 0:
            if (*p == (PROTOCOL_HEAD & 0xFF))
            {
                recv_msg.clear();
                recv_msg.push_back(PROTOCOL_HEAD & 0xFF);
                state = 1;
            }
            p++;
            data_len--;
            break;

        case 1:
            if (*p == ((PROTOCOL_HEAD >> 8) & 0xFF))
            {
                recv_msg.push_back(((PROTOCOL_HEAD >> 8) & 0xFF));
                p++;
                data_len--;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // len
            recv_msg.push_back(*p);
            len = *p;
            p++;
            data_len--;
            state = 3;
            break;

        case 3: // len
            recv_msg.push_back(*p);
            len += (*p) * 256;
            if (len > 1024 * 10)
            {
                state = 0;
                break;
            }
            p++;
            data_len--;
            state = 4;
            break;

        case 4: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 5;
            break;

        case 5: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 6;
            break;

        case 6: //
            if (len--)
            {
                recv_msg.push_back(*p);
                p++;
                data_len--;
            }
            else
            {
                state = 7;
            }
            break;

        case 7: {
            int i;
            uint8_t bcc = 0;
            recv_msg.push_back(*p);
            p++;
            data_len--;
            state = 0;

            for (i = 4; i < recv_msg.size(); i++)
            {
                bcc ^= recv_msg[i];
            }

            if (bcc == 0)
            {
                if (initDone_)
                {
                    tianbotDataProc(&recv_msg[0], recv_msg.size()); // process recv msg
                }
                communication_timer_.stop();                    // restart timer for communication timeout
                communication_timer_.start();
            }
            else
            {
                ROS_INFO("BCC error");
            }
            state = 0;
        }
        break;

        default:
            state = 0;
            break;
        }
    }
}

void TianbotCore::communicationErrorCallback(const ros::TimerEvent &)
{
    ROS_ERROR_THROTTLE(5, "Communication with base error");
}

void TianbotCore::heartCallback(const ros::TimerEvent &)
{
    vector<uint8_t> buf;
    uint16_t dummy = 0;

    buildCmd(buf, PACK_TYPE_HEART_BEAT, (uint8_t *)&dummy, sizeof(dummy));
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    heartbeat_timer_.start();
}

void TianbotCore::debugCmdCallback(const std_msgs::String::ConstPtr &msg)
{
    vector<uint8_t> buf;
    buildCmd(buf, PACK_TYPE_DEBUG, (uint8_t *)msg->data.c_str(), msg->data.length());
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
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

bool TianbotCore::debugCmdSrv(tianbot_core::DebugCmd::Request &req, tianbot_core::DebugCmd::Response &res)
{
    vector<uint8_t> buf;
    debugResultFlag_ = false;
    uint32_t count = 200;
    buildCmd(buf, PACK_TYPE_DEBUG, (uint8_t *)req.cmd.c_str(), req.cmd.length());
    if (comm_inf_->send(&buf[0], buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = NULL;
        ROS_ERROR("communication failed, reopen the device");
        heartbeat_timer_.stop();
        communication_timer_.stop();
        open();
        communication_timer_.start();
    }
    if (req.cmd == "reset")
    {
        res.result = "reset";
        return true;
    }
    else if (req.cmd == "param save" || req.cmd == "param reset")
    {
        count = 2000;
    }
    else if (req.cmd.find("set_") != req.cmd.npos) // adaptation for old racecar
    {
        count = 3000;
    }
    while (count-- && !debugResultFlag_)
    {
        ros::Duration(0.001).sleep();
    }
    if (debugResultFlag_)
    {
        res.result = debugResultStr_;
        return true;
    }
    else
    {
        return false;
    }
}

void TianbotCore::checkDevType(void)
{
    string type_keyword_list[] = {"base_type: ", "end"};
    string type;
    string dev_param;
    string dev_type;
    string::size_type start;
    string::size_type end;

    string cmd = "param get";
    vector<uint8_t> buf;
    uint32_t count;
    uint32_t retry;

    for (retry = 0; retry < 5; retry++)
    {
        debugResultFlag_ = false;
        count = 300;
        buf.clear();
        buildCmd(buf, PACK_TYPE_DEBUG, (uint8_t *)cmd.c_str(), cmd.length());
        if (comm_inf_->send(&buf[0], buf.size()) != 0)
        {
            delete comm_inf_;
            comm_inf_ = NULL;
            ROS_ERROR("communication failed, reopen the device");
            heartbeat_timer_.stop();
            communication_timer_.stop();
            open();
            communication_timer_.start();
        }
        while (count-- && !debugResultFlag_)
        {
            ros::Duration(0.001).sleep();
        }
        if (debugResultFlag_)
        {
            dev_param = debugResultStr_;
            break;
        }
        else
        {
            ROS_INFO("Get Device type failed, retry after 1s ...");
            ros::Duration(1).sleep();
        }
    }
    if (retry == 5)
    {
        ROS_ERROR("No valid device type found");
        return;
    }
    for (int i = 0; type_keyword_list[i] != "end"; i++)
    {
        start = dev_param.find(type_keyword_list[i]);
        if (start != dev_param.npos)
        {
            start += type_keyword_list[i].length();
            end = dev_param.find("\r\n", start);
            if (end == dev_param.npos)
            {
                end = dev_param.length();
            }
            dev_type = dev_param.substr(start, end - start);
            ROS_INFO("Get device type [%s]", dev_type.c_str());
            nh_.param<std::string>("type", type, DEFAULT_TYPE);
            if (dev_type == "omni" || dev_type == "mecanum")
            {
                dev_type = "omni";
            }
            if (type == dev_type)
            {
                ROS_INFO("Device type match");
            }
            else
            {
                ROS_ERROR("Device type mismatch, set [%s] get [%s]", type.c_str(), dev_type.c_str());
            }
            return;
        }
    }
    ROS_ERROR("No valid device type found");
}

void TianbotCore::open(void)
{
    std::string param_serial_port;
    std::string client_ip;
    if (!nh_.getParam("serial_port", param_serial_port) && !nh_.getParam("client_ip", client_ip))
    {
        ROS_FATAL("Please specify the serial_port or client_ip");
        exit(-1);
    }
    else if (nh_.getParam("serial_port", param_serial_port))
    {
        comm_inf_ = new Serial();
        struct serial_cfg s_cfg;
        nh_.param<int>("serial_baudrate", s_cfg.rate, DEFAULT_SERIAL_BAUDRATE);
        s_cfg.device = (char *)param_serial_port.c_str();
        s_cfg.databits = 8;
        s_cfg.flow_ctrl = 0;
        s_cfg.parity = 'N';
        s_cfg.stopbits = 1;
        //ROS_INFO("Using %s for communication, baudrate: %d", param_serial_port.c_str(), s_cfg.rate);
        while (comm_inf_->open(&s_cfg, boost::bind(&TianbotCore::dataProc, this, _1, _2)) != true)
        {
            if (!ros::ok())
                exit(0);
            ROS_ERROR_THROTTLE(5.0, "Device %s open failed", param_serial_port.c_str());
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Device %s open successfully", param_serial_port.c_str());
    }
    else if (nh_.getParam("client_ip", client_ip))
    {
        comm_inf_ = new Udp();
        struct udp_cfg u_cfg;
        nh_.param<int>("client_port", u_cfg.udp_send_port, DEFAULT_CLIENT_PORT);
        nh_.param<int>("server_port", u_cfg.udp_recv_port, DEFAULT_SERVER_PORT);
        u_cfg.client_addr = client_ip;
        while (comm_inf_->open(&u_cfg, boost::bind(&TianbotCore::dataProc, this, _1, _2)) != true)
        {
            ROS_ERROR_THROTTLE(5.0, "Lesten device %s:%d failed", client_ip.c_str(), u_cfg.udp_send_port);
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Listen device %s:%d, server port %d ", client_ip.c_str(), u_cfg.udp_send_port, u_cfg.udp_recv_port);
    }
}

TianbotCore::TianbotCore(ros::NodeHandle *nh) : nh_(*nh), initDone_(false)
{
    open();
    debug_result_pub_ = nh_.advertise<std_msgs::String>("debug_result", 1);
    debug_cmd_sub_ = nh_.subscribe("debug_cmd", 1, &TianbotCore::debugCmdCallback, this);
    param_set_ = nh_.advertiseService<tianbot_core::DebugCmd::Request, tianbot_core::DebugCmd::Response>("debug_cmd_srv", boost::bind(&TianbotCore::debugCmdSrv, this, _1, _2));
    heartbeat_timer_ = nh_.createTimer(ros::Duration(0.2), &TianbotCore::heartCallback, this);
    communication_timer_ = nh_.createTimer(ros::Duration(0.2), &TianbotCore::communicationErrorCallback, this);
    heartbeat_timer_.stop();
    communication_timer_.stop();
    
    heartbeat_timer_.start();
    communication_timer_.start();
}
