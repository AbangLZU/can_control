#include "ros/ros.h"
#include "canalystii_node.h"
#include "can_node_msg/can.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include "pid.h"
#include <iostream>
#include <fstream>

unsigned char control_type = 2;
VCI_BOARD_INFO pInfo; //用来获取设备信息。
int count = 0;        //数据列表中，用来存储列表序号。
int wheel_speed_control = 0;
int steering_angle_control = 0;
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

void cmd_velCallback(const geometry_msgs::TwistStamped &harry)
{
    double design_vel = harry.twist.linear.x;
    if (design_vel < -twist_speed_limit)
    {
        design_vel = -twist_speed_limit;
    }
    else if (design_vel > twist_speed_limit)
    {
        design_vel = twist_speed_limit;
    }

    if (design_vel < 0)
    {
        wheel_speed_control = -design_vel * 3.6;
        control_type = 3;
    }
    if (design_vel > 0)
    {
        wheel_speed_control = design_vel * 3.6;
        control_type = 1;
    }
    if (design_vel == 0)
    {
        wheel_speed_control = 0;
        control_type = 2;
    }
    jiaoyan = 0;
}

void steerangle_callback(const std_msgs::Float32 &harry)
{
    // harry.data: -550~550
    // steering_angle_control=-harry.data*154.947/12;
    // if(steering_angle_control>5888)steering_angle_control=5888;
    // if(steering_angle_control<-5888)steering_angle_control=-5888;
    // else steering_angle_control=steering_angle_control;

    // steering_angle_control=harry.data*6.06;
    steering_angle_control = harry.data * 9.81;
    if (steering_angle_control > 5400)
        steering_angle_control = 5400;
    if (steering_angle_control < -5400)
        steering_angle_control = -5400;
    else
        steering_angle_control = steering_angle_control;
    wheel_speed_control_debug = wheel_speed_control;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_control_node");
    CANalystii_node can_node;
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("twist_cmd", 50, cmd_velCallback);
    ros::Subscriber steering_control = nh.subscribe("desired_steerangle_cmd", 50, steerangle_callback);

    ///pid test
    // PID pid = PID(0.1, 10, -10, 0.1, 0.01, 0.5);
    ///

    if (!can_node.start_device())
    {
        ROS_WARN("device starts error");
        return -1;
    }
    ROS_INFO("listening to can bus");
    VCI_CAN_OBJ can_obj;
    printf(">>this is hello !\r\n"); //指示程序已运行

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode = 0x80000008;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;     //接收所有帧
    config.Timing0 = 0x00; /*波特率500 Kbps  0x03  0x1C*/
    config.Timing1 = 0x1C;
    config.Mode = 0; //正常模式
    unsigned int can_idx = 0;
    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config) != 1)
    {
        printf(">>Init CAN2 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
    {
        printf(">>Start CAN2 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    //需要发送的帧，结构体设置
    VCI_CAN_OBJ send[1];
    send[0].ID = 0;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 1;
    send[0].DataLen = 8;

    int i = 0;
    for (i = 0; i < send[0].DataLen; i++)
    {
        send[0].Data[i] = i;
    }

    unsigned int recv_len = 1;
    ros::Rate r(10.0);

    while (ros::ok())
    {
        jiaoyan += 1;
        if (jiaoyan >= 4)
            wheel_speed_control = 0;
        if (jiaoyan < 4)
            wheel_speed_control = wheel_speed_control;
        if (jiaoyan > 20)
            jiaoyan = 4;
        ros::spinOnce();
        unsigned int recv_len = 1;

        if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
        {
            send[0].ID = 387;
            send[0].TimeFlag = 1;
            send[0].SendType = 0;
            send[0].RemoteFlag = 0;
            send[0].ExternFlag = 0;
            send[0].DataLen = 8;
            harry_speed = wheel_speed_control;
            send[0].Data[0] = harry_speed; // km/h
            // send[0].Data[0]=10; //  km/h

            // 2288~13312  ==> 2400~13200 zhongzhi:7800  zuoyou ge :5400
            steering_angle_control_char = 7800 + steering_angle_control; //the old: 10496~22272
            // steering_angle_control_char = 7800;//7800

            send[0].Data[1] = steering_angle_control_char >> 8;
            send[0].Data[2] = steering_angle_control_char;

            send[0].Data[3] = control_type;
            // send[0].Data[3]=1;

            send[0].Data[4] = 1;
            send[0].Data[5] = 1;
            send[0].Data[6] = 1;
            send[0].Data[7] = send[0].Data[0] + send[0].Data[1] + send[0].Data[2] + send[0].Data[3] + send[0].Data[4] +
                              send[0].Data[5] + send[0].Data[6];
            harry6 = jiaoyan;

            ROS_INFO("speed: %d", send[0].Data[0]);
            ROS_INFO("steering angle is %d", steering_angle_control_char);
        }
        r.sleep();
    }

    return 0;
}
