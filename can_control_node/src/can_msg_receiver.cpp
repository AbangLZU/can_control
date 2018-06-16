#include "can_msg_receiver.h"

CanMsgReceiver::CanMsgReceiver(ros::NodeHandle &nh)
{

    nh.param<std::string>("can_msg_pub_topic", can_msg_pub_topic_, "/can_msg_received");
    ROS_INFO("can_msg_pub_topic_: %s", can_msg_pub_topic_.c_str());
    can_msg_pub_ = nh.advertise<can_node_msg::can>(can_msg_pub_topic_, 10);

    // init the param
    vci_device_type_ = 4;
    vci_device_ind_ = 0;
    can_port_idx_ = 0;
    wait_time = 20;

    // set the config structure
    config.AccCode = 0x80000008;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;     //接收所有帧
    config.Timing0 = 0x00; /*波特率500 Kbps  0x03  0x1C*/
    config.Timing1 = 0x1C;
    config.Mode = 0; //正常模式

    if (PrepareCan())
    {
        std::cout << "Prepare can successfully!!" << std::endl;
    }
}

CanMsgReceiver::~CanMsgReceiver()
{
}

int CanMsgReceiver::PrepareCan()
{
    if (VCI_OpenDevice(vci_device_type_, vci_device_ind_, 0) != 1)
    {
        ROS_ERROR("Can not open the can device!!");
        return 0;
    }
    if (VCI_InitCAN(vci_device_type_, vci_device_ind_, can_port_idx_, &config) != 1)
    {
        ROS_ERROR(">>Init CAN %d error\n", can_port_idx_);
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 0;
    }
    if (VCI_StartCAN(vci_device_type_, vci_device_ind_, can_port_idx_) != 1)
    {
        ROS_ERROR(">>Start CAN %d error\n", can_port_idx_);
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 0;
    }

    return 1;
}

can_node_msg::can CanMsgReceiver::can_obj2msg(const VCI_CAN_OBJ &can_obj)
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

void CanMsgReceiver::Spin(int freq)
{
    ros::Rate r(freq);
    while (ros::ok())
    {
        process();
        ros::spinOnce();
        r.sleep();
    }
}

void CanMsgReceiver::process()
{
    // before receive the can msg, check the msg number in the receive buffer, we get all
    // buffered msgs at one time
    DWORD buffered_frame_num = VCI_GetReceiveNum(vci_device_type_, vci_device_ind_, can_port_idx_);
    if (buffered_frame_num > 100)
    {
        // if the buffered msg num is more than the structure size
        buffered_frame_num = 100;
    }

    unsigned int receive_len = 0;

    receive_len = VCI_Receive(vci_device_type_, vci_device_ind_, can_port_idx_, can_obj, buffered_frame_num, wait_time);

    if (receive_len)
    {
        // here we search the msg that we are interested
        for (int j = 0; j < buffered_frame_num; j++)
        {
            if (can_obj[j].ID == 388)
            {
                // when we received the msg we interested, to msg and publish
                can_node_msg::can msg = can_obj2msg(can_obj[j]);
                can_msg_pub_.publish(msg);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_msg_receiver");

    ros::NodeHandle nh;

    CanMsgReceiver can_receiver(nh);
    can_receiver.Spin(50);
    return 0;
}