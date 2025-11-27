#ifndef _PROTOCOL_
#define _PROTOCOL_

#include "stdint.h"
#include <vector>

using namespace std;

#define PROTOCOL_HEAD 0xAA55

enum
{
    PACK_TYPE_HEART_BEAT = 0x0000,
    PACK_TYPE_CMD_VEL,
    PACK_TYPE_ACKMAN_VEL,
    PACK_TYPE_SIGNAL_CTRL,
    PACK_TYPE_ACTUATOR_CTRL,
    PACK_TYPE_HAITAI_CTRL,  // 添加海泰电机控制包类型
    PACK_TYPE_LINE_OPTO_CTRL, /* 上位机触发换行光耦控制（uint32_t pulse_ms） */
    PACK_TYPE_EMM_V5_CTRL,  // 添加EmmV5电机控制包类型
    PACK_TYPE_SET_ROVER_MOTION_MODE,
    PACK_TYPE_DEBUG = 0x4000,
    PACK_TYPE_ODOM_RESPONSE = 0x8000,
    PACK_TYPE_UWB_RESPONSE,
    PACK_TYPE_HEART_BEAT_RESPONSE,
    PACK_TYPE_IMU_REPONSE,
    PACK_TYPE_ODOM_V2_RESPONSE,
    PACK_TYPE_DEBUG_RESPONSE = 0xC000,
    PACK_TYPE_Voltage_RESPONSE,
    PACK_TYPE_SIGNAL_STATUS,
    PACK_TYPE_ACTUATOR_STATUS,
    PACK_TYPE_HAITAI_VELOCITY,  // 添加海泰电机速度反馈包类型
    PACK_TYPE_EMM_V5_VELOCITY,

};


struct vector3
{
    float x;
    float y;
    float z;
};

struct twist
{
    struct vector3 linear;
    struct vector3 angular;
};

struct quaternion
{
    float x;
    float y;
    float z;
    float w;
};

struct pose
{
    struct vector3 point;
    float yaw;
};

struct pose_v2
{
    struct vector3 point;
    struct vector3 rpy;
};

struct odom
{
    struct pose pose;
    struct twist twist;
};

struct odom_v2
{
    struct pose_v2 pose;
    struct twist twist;
};

struct uwb
{
    float x_m;
    float y_m;
    float yaw;
    // uint32_t sig_level;
};

struct imu_feedback
{
    struct quaternion quat;
    struct vector3 linear_acc;
    struct vector3 angular_vel;
};

struct ackermann_cmd
{
    float steering_angle;
    float speed;
};

struct voltage
{
    float Battery_voltage;
};

struct motion_mode
{
    uint32_t mode;
};

struct signal_status
{
    uint8_t red;
    uint8_t yellow;
    uint8_t green;
    uint8_t buzzer;
};

struct actuator_status
{
    uint8_t state;
};
/* Command structure for line opto: pulse duration in milliseconds (uint32_t) */
struct line_opto_cmd {
    uint32_t pulse_ms;
};
struct protocol_pack
{
    uint16_t head;
    uint16_t len; // data len + 2 byte pack_type
    uint16_t pack_type;
    uint8_t data[]; // contain bcc byte
};
struct HaitaiCtrl_t{
    float position;    // 目标位置 (rad)
    float velocity;    // 目标速度 (rad/s)
    float torque;      // 直接力矩 (N·m)
    float kp;          // 位置增益
    float kd;          // 速度增益
};
struct haitai_vel
{
    float velocity;    // 当前速度 (rad/s)
};
// 定义EMMV5电机控制参数结构体
struct EmmV5Ctrl_t{
    float distance_mm;          // 距离（毫米）
    uint16_t vel;               // 速度
    uint8_t acc;                // 加速度
};
// 添加步进电机位置数据结构
struct EmmV5_vel{
    float position;          // 距离值（mm）
};


void buildCmd(vector<uint8_t> &buf, uint16_t cmd, uint8_t data[], uint8_t data_len);

#endif
