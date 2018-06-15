//
// Created by abang on 18-6-15.
//

#ifndef PROJECT_TWIST_CONVERTER_H
#define PROJECT_TWIST_CONVERTER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <coms.h>

/*To convert ros twist message's anglular velocity (rad/s) to desired steering angle in degree*/
/*To convert ros twist message's linear velocity (m/s) to desird car velocity in m/s*/

class twist_converter {

public:
    twist_converter(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);
    virtual ~twist_converter();

    void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg);

    double convert_trans_rot_vel_to_wheel_angle(double linear_v, double angular_v, double wheelbase);

    virtual void spin();
    virtual void spinonce();
    void process();

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    //parameters
    std::string twist_topic_;
    std::string desired_steerangle_topic_;
    std::string desired_velocity_topic_;

    //published topics
    std_msgs::Float32 desired_steerangle_;  // degree
    std_msgs::Float32 desired_velocity_;    // m/s

    //subscriber
    ros::Subscriber twist_subscriber;
    //publisher
    ros::Publisher desired_steerangle_publisher;
    ros::Publisher desired_velocity_publisher;

    //car paramters
    double wheelbase;
    double track;
    double steerratio;
    double maxsteerangle;
    double minsteerangle;
    double maxvelocity;
    double minvelocity;
};


#endif //PROJECT_TWIST_CONVERTER_H
