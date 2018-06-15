
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

double vx=0;
double vz=0;
double vx1=0;
double vz1=0;
double image_flag_go=0;
double image_flag_stop=0;
double jiaoyan=0;

void cmd_velCallback(const geometry_msgs::TwistStamped &harry)
{
  vx1=harry.twist.linear.x;
  vz1=harry.twist.angular.z;
  if(vz1<0.05&&vz1>-0.05)vz1=0;
}
void image_control_callback(const geometry_msgs::TwistStamped &harry)
{
  if(harry.twist.linear.x==2&&harry.twist.angular.z==0)image_flag_stop=1;
  else image_flag_stop=2;
}
void cmd_velCallback1(const geometry_msgs::Twist &harry)
{
  vx=harry.linear.x;
  vz=harry.angular.z;
  jiaoyan=0;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "canbus_trans");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 50);
  ros::Subscriber control_sub2 = nh.subscribe("cmd_vel", 50, cmd_velCallback1);
  ros::Subscriber control_sub = nh.subscribe("twist_cmd1", 50, cmd_velCallback);
  ros::Subscriber control_sub1 = nh.subscribe("image_control", 50, image_control_callback);

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    jiaoyan+=1;
    if(jiaoyan>65536)jiaoyan=5;
    ROS_INFO("jiaoyan  is %f", jiaoyan);
    ros::spinOnce();
    geometry_msgs::TwistStamped  twist_cmd11;
    if(jiaoyan<5)
    {

     
      twist_cmd11.twist.linear.x=vx;
      twist_cmd11.twist.linear.y=0;
      twist_cmd11.twist.linear.z=0;
      twist_cmd11.twist.angular.x=0;
      twist_cmd11.twist.angular.y=0;
      twist_cmd11.twist.angular.z=vz;
     


    }
    else
    {

      twist_cmd11.twist.linear.x=vx1;
      twist_cmd11.twist.linear.y=0;
      twist_cmd11.twist.linear.z=0;
      twist_cmd11.twist.angular.x=0;
      twist_cmd11.twist.angular.y=0;
      twist_cmd11.twist.angular.z=vz1;

    }
    cmd_vel_pub.publish(twist_cmd11);
    loop_rate.sleep();
    }

}
