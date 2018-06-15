#pragma once

#include "ros/ros.h"
#include "controlcan.h"
#include "can_node_msg/can.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include "pid.h"
#include <iostream>
#include <fstream>

class CanMsgReceiver{
    private:
        VCI_CAN_OBJ can_obj[100];

        //初始化参数，严格参数二次开发函数库说明书。
        VCI_INIT_CONFIG config;

        int vci_device_type_;
        int vci_device_ind_;
        unsigned int can_port_idx_;
        int wait_time; // when receive buffer is empty, wait for 20 ms

        std::string can_msg_pub_topic_;

        ros::Publisher can_msg_pub_;

        int PrepareCan();

        void process();

        can_node_msg::can can_obj2msg(const VCI_CAN_OBJ& can_obj);

    public:
        CanMsgReceiver(ros::NodeHandle &nh);
        virtual ~CanMsgReceiver();

        void Spin(int freq);

};