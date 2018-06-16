#pragma once

#include "ros/ros.h"
#include "controlcan.h"
#include "can_node_msg/can.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include "pid.h"
#include "coms.h"
#include <iostream>
#include <fstream>

class CanMsgSender
{
  private:
    VCI_CAN_OBJ send_frame[3];

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;

    int vci_device_type_;
    int vci_device_ind_;
    unsigned int can_port_idx_;
    int wait_time; // when receive buffer is empty, wait for 20 ms

    double design_vel = 0.0;
    double design_steerling = 0.0;

    unsigned char control_type = 2;
    int count = 0;        //数据列表中，用来存储列表序号。
    int harry_speed = 0;
    int harry1 = 0;
    int harry2 = 0;
    int steering_angle_control_char = 16384;
    double harry6 = 0;
    double jiaoyan = 0;
    double wheel_speed_control_debug = 0;
    double steering_angle_control_debug = 0;

    double twist_speed_limit = 4.0; //2.78; //m/s
    int current_speed = 0;          //km/s reed from the can
    double pid_result = 0.0;

    // int PrepareCan();

    void process();

    can_node_msg::can can_obj2msg(const VCI_CAN_OBJ &can_obj);

    void CmdvelCallback(const geometry_msgs::TwistStamped &twist_msg);

    void SteerangleCallback(const std_msgs::Float32 &harry);

    void SetSteerData();
    void SetBreakData();
    void SetThrottleData();

  public:
    CanMsgSender(ros::NodeHandle &nh_);
    virtual ~CanMsgSender();

    void Spin(int freq);
};