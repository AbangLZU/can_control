#include "can_msg_sender.h"

CanMsgSender::CanMsgSender(ros::NodeHandle &nh_)
{

    ros::Subscriber cmd_vel_sub = nh_.subscribe("twist_cmd", 50, &CanMsgSender::CmdvelCallback, this);
    ros::Subscriber steering_control = nh_.subscribe("desired_steerangle_cmd", 50, &CanMsgSender::SteerangleCallback, this);

    // init the param
    vci_device_type_ = 4;
    vci_device_ind_ = 0;
    can_port_idx_ = 0;
    wait_time = 20;

    // set the config structure
    // config.AccCode = 0x80000008;
    // config.AccMask = 0xFFFFFFFF;
    // config.Filter = 1;     //接收所有帧
    // config.Timing0 = 0x00; /*波特率500 Kbps  0x03  0x1C*/
    // config.Timing1 = 0x1C;
    // config.Mode = 0; //正常模式

    // if (PrepareCan())
    // {
    //     std::cout << "Prepare can successfully!!" << std::endl;
    // }
}

CanMsgSender::~CanMsgSender()
{
}

// int CanMsgSender::PrepareCan()
// {
//     bool can_device_open;
//     bool can_device_init;
//     bool can_device_start;
//     if (ros::param::get("/can_device_open", can_device_open))
//     {
//         if (!can_device_open)
//         {
//             if (VCI_OpenDevice(vci_device_type_, vci_device_ind_, 0) != 1)
//             {
//                 ROS_ERROR("Can not open the can device!!");
//                 return 0;
//             }
//             ros::param::set("/can_device_open", true);
//         }
//     }
//     if (ros::param::get("/can_device_init", can_device_init))
//     {
//         if (!can_device_init)
//         {
//             if (VCI_InitCAN(vci_device_type_, vci_device_ind_, can_port_idx_, &config) != 1)
//             {
//                 ROS_ERROR(">>Init CAN %d error\n", can_port_idx_);
//                 VCI_CloseDevice(VCI_USBCAN2, 0);
//                 return 0;
//             }
//             ros::param::set("/can_device_init", true);
//         }
//     }
//     if (ros::param::get("/can_device_start", can_device_start))
//     {
//         if (!can_device_start)
//         {
//             if (VCI_StartCAN(vci_device_type_, vci_device_ind_, can_port_idx_) != 1)
//             {
//                 ROS_ERROR(">>Start CAN %d error\n", can_port_idx_);
//                 VCI_CloseDevice(VCI_USBCAN2, 0);
//                 return 0;
//             }
//         }
//     }
//     return 1;
// }

can_node_msg::can CanMsgSender::can_obj2msg(const VCI_CAN_OBJ &can_obj)
{
    can_node_msg::can can_msg;
    //set Header with ros time rather than can_obj time, in case there are no timestamp
    //TODO: miss seq here
    can_msg.header.frame_id = "/canalystii";
    can_msg.header.stamp = ros::Time::now();

    //set data
    can_msg.id = can_obj.ID;
    can_msg.timeflag = can_obj.TimeFlag;
    can_msg.sendtype = can_obj.SendType;
    can_msg.remoteflag = can_obj.RemoteFlag;
    can_msg.externflag = can_obj.ExternFlag;
    can_msg.datalen = can_obj.DataLen;
    for (int i = 0; i < 8; i++)
    {
        can_msg.data[i] = can_obj.Data[i];
    }
    return can_msg;
}

void CanMsgSender::Spin(int freq)
{
    ros::Rate r(freq);
    while (ros::ok())
    {
        // 先更新，再发送
        ros::spinOnce();
        process();

        r.sleep();
    }
}

void CanMsgSender::process()
{
    for (int i = 0; i < 3; i++)
    {
        send_frame[i].SendType = 0;
        send_frame[i].RemoteFlag = 0;
        send_frame[i].ExternFlag = 0;
        send_frame[i].DataLen = 8;
    }

    if (VCI_Transmit(vci_device_type_, vci_device_ind_, can_port_idx_, send_frame, 3) == 1)
    {
        ROS_INFO("Transmit successfully!!");
    }
}

void CanMsgSender::CmdvelCallback(const geometry_msgs::TwistStamped &twist_msg)
{
    double tmp_vel = twist_msg.twist.linear.x;
    if (tmp_vel < -twist_speed_limit)
    {
        tmp_vel = -twist_speed_limit;
    }
    else if (tmp_vel > twist_speed_limit)
    {
        tmp_vel = twist_speed_limit;
    }

    design_vel = tmp_vel * 3.6; //from m/s to km/h

    // if (design_vel < 0)
    // {
    //     wheel_speed_control = -design_vel * 3.6;
    //     control_type = 3;
    // }
    // if (design_vel > 0)
    // {
    //     wheel_speed_control = design_vel * 3.6;
    //     control_type = 1;
    // }
    // if (design_vel == 0)
    // {
    //     wheel_speed_control = 0;
    //     control_type = 2;
    // }
}

void CanMsgSender::SteerangleCallback(const std_msgs::Float32 &steer)
{
    // harry.data: -550~550
    // steering_angle_control=-harry.data*154.947/12;
    // if(steering_angle_control>5888)steering_angle_control=5888;
    // if(steering_angle_control<-5888)steering_angle_control=-5888;
    // else steering_angle_control=steering_angle_control;

    // TODO: the relationship between the real angle to the signal angle
    design_steerling = steer.data * 9.81;
    if (design_steerling > 5400)
        design_steerling = 5400;
    if (design_steerling < -5400)
        design_steerling = -5400;
}

void CanMsgSender::SetSteerData()
{
    // the first frame is the steering frame
    send_frame[0].ID = STEERLING_TX;

    //TODO: set the data of steering frame
}

void CanMsgSender::SetBreakData()
{
    // the second frame is the break frame
    send_frame[1].ID = BREAK_TX;

    //TODO: set the data of break frame
}
void CanMsgSender::SetThrottleData()
{
    // the third frame is the throttle frame
    send_frame[2].ID = THROTTLE_TX;

    //TODO: set the data of throttle frame
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_msg_sender");
    ros::NodeHandle nh;

    CanMsgSender sender(nh);
    // the send frequency is 10 Hz
    sender.Spin(10);

    return 0;
}
