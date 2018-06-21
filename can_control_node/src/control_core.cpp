#include "control_core.h"

ControlCore::ControlCore()
    : private_nh_("~"),
      sys_enable_(false),
      control_gap_(1.0 / LOOP_RATE)
{
    ROS_INFO("DBW_node launch...initForROS...");
    initForROS();
}

ControlCore::~ControlCore()
{
}

void ControlCore::initForROS()
{
    srv_.setCallback(boost::bind(&ControlCore::cbFromDynamicReconfig, this, _1, _2));

    private_nh_.param<double>("vehicle_mass", vehicle_mass_, 1736.35);
    private_nh_.param<double>("fuel_capacity", fuel_capacity_, 13.5);
    private_nh_.param<double>("brake_deadband", brake_deadband_, 0.1);
    private_nh_.param<double>("decel_limit", decel_limit_, -5.0); //default: -5.0
    private_nh_.param<double>("accel_limit", accel_limit_, 1.0);  //default: 1.0
    private_nh_.param<double>("wheel_radius", wheel_radius_, 0.2413);
    private_nh_.param<double>("wheel_base", wheel_base_, 2.8498);
    private_nh_.param<double>("steer_ratio", steer_ratio_, 14.8);
    private_nh_.param<double>("max_lat_accel", max_lat_accel_, 3.0);
    private_nh_.param<double>("max_steer_angle", max_steer_angle_, 8.0);
    private_nh_.param<double>("fuel_level", fuel_level_, -1.0);

    //set basic parameters for yaw controller.
    yaw_controller_.setParameters(wheel_base_, steer_ratio_, max_lat_accel_, max_steer_angle_);

    //steer, throttle, brake publisher
    steer_pub_ = nh_.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 2);
    throttle_pub_ = nh_.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 2);
    brake_pub_ = nh_.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 2);

    //twist_cmd, dbw_enabled, current_velocity,steering_report subscriber
    sub_vel_ = nh_.subscribe("/twist_cmd", 2, &ControlCore::cbFromTwistCmd, this);
    sub_enable_ = nh_.subscribe("/vehicle/dbw_enabled", 1, &ControlCore::cbFromRecvEnable, this);
    sub_cur_vel_ = nh_.subscribe("/current_velocity", 2, &ControlCore::cbFromCurrentVelocity, this);
    sub_steer_report_ = nh_.subscribe("/vehicle/steering_report", 2, &ControlCore::cbFromSteeringReport, this);
    sub_fuel_report_ = nh_.subscribe("/vehicle/fuel_level_report", 2, &ControlCore::cbFromFuelLevelReport, this);

    //give a general value for MKZ, MKZ gas container is 63.
    lpf_fuel_.setParams(63.0, 0.02);

    //in simulation env, fuel_level is 50% assumed, in real env,
    //fuel_level is filled by callback function.
    if (fuel_level_ > 0)
    {
        //assume 50% Fuel level
        lpf_fuel_.filt(fuel_level_);

        //ROS_INFO("DBW_node, lpf_fuel_.get(): %f", lpf_fuel_.get());
        //ROS_INFO("DBW_node, lpf_fuel_.getReady(): %d", lpf_fuel_.getReady());
    }

    //lowpass accel,
    lpf_accel_.setParams(0.5, 0.02);

    //set speed pid range for tune
    speed_pid_.setRange(-MAX_THROTTLE_PERCENTAGE, MAX_THROTTLE_PERCENTAGE);
    accel_pid_.setRange(0, MAX_THROTTLE_PERCENTAGE);
}

void ControlCore::run()
{
    ros::Rate loop_rate(LOOP_RATE); //50hz

    while (ros::ok())
    {
        ros::spinOnce();

        if (sys_enable_ == true)
        {
            //get controller's value, including throttle, steering, brake
            getPredictedControlValues();
        }
        else
        /*if enable is false, means manually drive, should reset PID parameter to avoid accumulate error.*/
        {
            speed_pid_.resetError();
            accel_pid_.resetError();
        }

        loop_rate.sleep();
    }
}

/* Publish control command */
void ControlCore::publishControlCmd(Controller v_controller)
{
    dbw_mkz_msgs::ThrottleCmd tcmd = dbw_mkz_msgs::ThrottleCmd();
    tcmd.enable = true;
    // throttle is percentage
    tcmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
    tcmd.pedal_cmd = v_controller.throttle;
    this->throttle_pub_.publish(tcmd);

    dbw_mkz_msgs::SteeringCmd scmd = dbw_mkz_msgs::SteeringCmd();
    scmd.enable = true;
    scmd.steering_wheel_angle_cmd = v_controller.steer;
    this->steer_pub_.publish(scmd);

    dbw_mkz_msgs::BrakeCmd bcmd = dbw_mkz_msgs::BrakeCmd();
    bcmd.enable = true;
    //brake is torque (N*m)
    bcmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
    bcmd.pedal_cmd = v_controller.brake;
    this->brake_pub_.publish(bcmd);
}

/* Listen dbw_enable message */
void ControlCore::cbFromRecvEnable(const std_msgs::Bool::ConstPtr &msg)
{
    sys_enable_ = msg->data;
}

/* Listen twist_cmd message, call back function */
void ControlCore::cbFromTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    twist_cmd_.header = msg->header;
    twist_cmd_.twist = msg->twist;
}

/* Listen cur_velocity message, which is from simulator, call back function */
void ControlCore::cbFromCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    cur_velocity_.header = msg->header;
    cur_velocity_.twist = msg->twist;
}

/* Listen SteeringReport message, which is actual value, call back function */
void ControlCore::cbFromSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg)
{
    double raw_accel = LOOP_RATE * (msg->speed - cur_velocity_.twist.linear.x);
    lpf_accel_.filt(raw_accel);

    if (fabs(msg->speed - cur_velocity_.twist.linear.x) < (double)1e-2)
    {
        cur_velocity_.twist.linear.x = msg->speed;
    }
}

/* Listen fuel_level_report message, call back function */
void ControlCore::cbFromFuelLevelReport(const dbw_mkz_msgs::FuelLevelReport::ConstPtr &msg)
{
    lpf_fuel_.filt(msg->fuel_level);
}

/* Listen Dynamic Reconfig message, which is actual value, call back function */
void ControlCore::cbFromDynamicReconfig(twist_controller::ControllerConfig &config, uint32_t level)
{
    cfg_ = config;
    ROS_INFO("Reconfigure Request: speed_kp: %f, speed_ki: %f, speed_kd: %f", cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd);
    ROS_INFO("Reconfigure Request: accel_kp: %f, accel_ki: %f, accel_kd: %f", cfg_.accel_kp, cfg_.accel_ki, cfg_.accel_kd);

    //set speed pid's P/I/D
    speed_pid_.setGains(cfg_.speed_kp, cfg_.speed_ki, cfg_.speed_kd);
    //set acceleration pid's P/I/D
    accel_pid_.setGains(cfg_.accel_kp, cfg_.accel_ki, cfg_.accel_kd);
}

/* get controller's value, including throttle, steering, brake */
void ControlCore::getPredictedControlValues()
{
    // vehicle mass calculation
    double vehicle_mass = vehicle_mass_ + lpf_fuel_.get() / 100.0 * fuel_capacity_ * GAS_DENSITY;

    double vel_cte = twist_cmd_.twist.linear.x - cur_velocity_.twist.linear.x;

    if (fabs(twist_cmd_.twist.linear.x) < mphToMps(1.0))
    {
        speed_pid_.resetError();
    }

    //speed pid, get adjusted speed_cmd
    double speed_cmd = speed_pid_.step(vel_cte, control_gap_);

    if (twist_cmd_.twist.linear.x <= (double)1e-2)
    {
        speed_cmd = std::min(speed_cmd, -530 / vehicle_mass / wheel_radius_);
    }

    //set throttle signal
    if (speed_cmd >= 0.0)
    {
        vehicle_controller_.throttle = accel_pid_.step(speed_cmd - lpf_accel_.get(), control_gap_);
    }
    else
    {
        accel_pid_.resetError();
        vehicle_controller_.throttle = 0.0;
    }

    //set brake signal
    if (speed_cmd < -brake_deadband_)
    {
        //get vehicle brake torque, T = m * a * r
        vehicle_controller_.brake = -speed_cmd * vehicle_mass * wheel_base_ / 9.8;
        //ROS_INFO("vehicle_controller_.brake: %f", vehicle_controller_.brake);
        if (vehicle_controller_.brake > TORQUE_MAX)
        {
            vehicle_controller_.brake = TORQUE_MAX;
        }
    }
    else
    {
        vehicle_controller_.brake = 0;
    }

    //set steering signal, only use kp controller
    double steering_wheel = yaw_controller_.get_steering(twist_cmd_.twist.linear.x, twist_cmd_.twist.angular.z,
                                                         cur_velocity_.twist.linear.x);
    vehicle_controller_.steer = steering_wheel + cfg_.steer_kp * (twist_cmd_.twist.angular.z - cur_velocity_.twist.angular.z);

    //TODO:: for simulator test to see work or not, remember to remove
    //vehicle_controller_.throttle = 0.3;
    //vehicle_controller_.steer = 0.0;
    //vehicle_controller_.brake = 0;

    publishControlCmd(vehicle_controller_);

    vehicle_controller_.reset();
}
