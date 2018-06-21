#include "twist_converter.h"

twist_converter::twist_converter(ros::NodeHandle &nh, ros::NodeHandle &nhPrivate)
{
    nh.param("twist_topic_", twist_topic_, std::string("twist_cmd"));                                        // in topic name
    nh.param("desired_steerangle_topic_", desired_steerangle_topic_, std::string("desired_steerangle_cmd")); // out topic name
    nh.param("desired_throttle_topic_", desired_throttle_topic_, std::string("desired_throttle_cmd"));       // out topic name
    nh.param("desired_break_topic_", desired_break_topic_, std::string("desired_break_cmd"));                // out topic name

    // initalize car constant
    wheelbase = WHEEL_BASE;                                    // meter
    track = WHEEL_TRACK;                                       // meter
    steerratio = WHEEL_TO_STEERING;                            // unitless
    maxsteerangle = STEERING_ANGLE_LIMIT / WHEEL_TO_STEERING;  // degree, CW for +
    minsteerangle = -STEERING_ANGLE_LIMIT / WHEEL_TO_STEERING; // degree, CCW for -
    maxvelocity = SPEED_LIMIT;                                 // m/s, IMPORTANT: this is safty limit
    minvelocity = 0.0;                                         // m/s

    sub_twist_ = nh.subscribe<geometry_msgs::TwistStamped>(twist_topic_, 1, boost::bind(&twist_converter::twist_cb, this, _1));
    sub_enable_ = nh.subscribe<std_msgs::Bool>("/vehicle/dbw_enabled", 1, &twist_converter::drive_state_cb, this);
    sub_cur_vel_ = nh.subscribe<geometry_msgs::TwistStamped>("/current_velocity", 2, &twist_converter::current_vel_cb, this);
    sub_steer_report_ = nh.subscribe<>("/vehicle/steering_report", 2, &twist_converter::steering_report_cb, this);
    sub_fuel_report_ = nh.subscribe<>("/vehicle/fuel_level_report", 2, &twist_converter::fuel_level_cb, this);

    desired_steerangle_publisher = nh.advertise<std_msgs::Float32>(desired_steerangle_topic_, 100);
    desired_throttle_publisher = nh.advertise<std_msgs::Float32>(desired_throttle_topic_, 100);
    desired_break_publisher = nh.advertise<std_msgs::Float32>(desired_break_topic_, 100);
}

twist_converter::~twist_converter()
{
    sub_twist_.shutdown();
}

void twist_converter::spin()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        spinonce();
        rate.sleep();
    }
}

void twist_converter::spinonce()
{
    process();
    ros::spinOnce();
}

void twist_converter::process()
{
    double carbody_steer_angle = convert_trans_rot_vel_to_wheel_angle(twist_.twist.linear.x, twist_.twist.angular.z, wheelbase);
    desired_steerangle_.data = carbody_steer_angle * steerratio;

    // TODO: PID controller for throttle and break
    desired_velocity_.data = std::min(std::max(twist_linear_v, minvelocity), maxvelocity);

    desired_steerangle_publisher.publish(desired_steerangle_);
    desired_velocity_publisher.publish(desired_velocity_);
}

void twist_converter::twist_cb(const geometry_msgs::TwistStamped::ConstPtr &twist_msg)
{
    twist_.header = twist_msg->header;
    twist_.twist = twist_msg->twist;
}

void twist_converter::drive_state_cb(const std_msgs::Bool::ConstPtr &msg){
    sys_enable_ = msg->data;
}

void twist_converter::current_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    current_velocity_.header = msg->header;
    current_velocity_.twist = msg->twist;
}
void twist_converter::steering_report_cb(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg){} 

void twist_converter::fuel_level_cb(const dbw_mkz_msgs::FuelLevelReport::ConstPtr &msg){}

double twist_converter::convert_trans_rot_vel_to_wheel_angle(double linear_v, double angular_v, double wheelbase)
{
    if (angular_v == 0 || linear_v == 0)
        return 0;
    double temp = angular_v / linear_v * wheelbase;
    return std::min(std::max(-atan(temp) / PI * 180, minsteerangle), maxsteerangle); //
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_converter_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    twist_converter *tc = 0;
    tc = new twist_converter(nh, nhPrivate);

    tc->spin();

    return 0;
}
