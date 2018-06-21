/*
 *  Copyright (c) 2018, Udacity
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/FuelLevelReport.h>
#include "yaw_controller.h"
#include "PIDControl.h"
#include "lowpassfilter.h"

#include <dynamic_reconfigure/server.h>
#include <twist_controller/ControllerConfig.h>

#define LOOP_RATE (50)
#define TORQUE_MAX (3412)             //Nm, maximum torque
#define MAX_THROTTLE_PERCENTAGE (0.8) //max throttle percentage

typedef struct Controller
{
    double throttle; // throttle
    double brake;    // brake
    double steer;    // steer

    Controller() : throttle(0.0), brake(0.0), steer(0.0) {}
    void reset()
    {
        throttle = 0.0;
        brake = 0.0;
        steer = 0.0;
    }
} Controller;

class ControlCore
{
  public:
    ControlCore();
    ~ControlCore();
    void run();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher throttle_pub_, brake_pub_, steer_pub_;
    ros::Subscriber sub_vel_, sub_cur_vel_, sub_enable_, sub_steer_report_, sub_fuel_report_;

    void initForROS();
    void cbFromTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void cbFromRecvEnable(const std_msgs::Bool::ConstPtr &msg);
    void cbFromCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void cbFromSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg);
    //dynamic re-configure
    void cbFromDynamicReconfig(twist_controller::ControllerConfig &config, uint32_t level);
    void cbFromFuelLevelReport(const dbw_mkz_msgs::FuelLevelReport::ConstPtr &msg);

    void getPredictedControlValues();
    void publishControlCmd(Controller v_controller);

    double vehicle_mass_, fuel_capacity_, brake_deadband_, decel_limit_, accel_limit_, wheel_radius_, wheel_base_, steer_ratio_, max_lat_accel_, max_steer_angle_, fuel_level_;
    bool sys_enable_;
    double control_gap_;
    //float throttle_, brake_, steer_;

    PIDControl speed_pid_; //pid for speed
    PIDControl accel_pid_; //pid for acceleration

    YawController yaw_controller_;
    Controller vehicle_controller_;

    LowPassFilter lpf_fuel_;
    LowPassFilter lpf_accel_;
    twist_controller::ControllerConfig cfg_;
    dynamic_reconfigure::Server<twist_controller::ControllerConfig> srv_;

    geometry_msgs::TwistStamped twist_cmd_;
    geometry_msgs::TwistStamped cur_velocity_;

    static const double GAS_DENSITY = 2.858;                     // for kg/gal transform
    static double mphToMps(double mph) { return mph * 0.44704; } //mph to m/s
};
