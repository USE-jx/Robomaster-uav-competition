#include "plan_and_control/trajectory_tracking_node.h"

GeometricControllerNode::GeometricControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) 
    : nh_(nh), nh_private_(nh_private), states_cmd_updated_(false),states_cmd_init_(false) {
    
    control_cmd_pub_ = 
        nh_.advertise<uav_msgs::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame",1);
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",1, &GeometricControllerNode::odomCallback,this);
    desired_states_sub_ =
        nh_.subscribe<uav_msgs::DesiredStates>("/reference/desiredStates", 10, &GeometricControllerNode::desiredStatesCallback, this);

    //get parameters from parameters sever
    nh_private_.getParam("position_gain/x", geometricController_.position_gain_.x());
    nh_private_.getParam("position_gain/y", geometricController_.position_gain_.y());
    nh_private_.getParam("position_gain/z", geometricController_.position_gain_.z());
    nh_private_.getParam("velocity_gain/x", geometricController_.velocity_gain_.x());
    nh_private_.getParam("velocity_gain/y", geometricController_.velocity_gain_.y());
    nh_private_.getParam("velocity_gain/z", geometricController_.velocity_gain_.z());
    nh_private_.getParam("attitude_gain/x", geometricController_.attitude_gain_.x());
    nh_private_.getParam("attitude_gain/y", geometricController_.attitude_gain_.y());
    nh_private_.getParam("attitude_gain/z", geometricController_.attitude_gain_.z());
    nh_private_.getParam("vehicle_mass", geometricController_.vehicle_mass_);
    
}

GeometricControllerNode::~GeometricControllerNode() { }

void GeometricControllerNode::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);

    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                msg->twist.twist.linear.y,
                                msg->twist.twist.linear.z);

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    // Eigen::Matrix3d odom_matrix (odom_orient_);
    // Eigen::Vector3d currentYPR = odom_matrix.eulerAngles(2,1,0);
    
    geometricController_.setOdometry(odom_pos_, odom_vel_, odom_orient_);
    //当期望状态发送停止后也一直发送命令使飞机维持在最后的期望状态
    if (states_cmd_init_) {
        if (!states_cmd_init_) {
            //pubRPYTCmd();
            pubRrPrYrTCmd();
        }
        states_cmd_updated_ = false;
    }

}

void GeometricControllerNode::desiredStatesCallback(const uav_msgs::DesiredStatesConstPtr &msg) {

    desired_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    desired_vel_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    desired_acc_ = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    desired_yaw_ = msg->yaw;
    geometricController_.setDesiredStates(desired_pos_, desired_vel_,desired_acc_,desired_yaw_);
    
    pubRrPrYrTCmd();
    states_cmd_init_ = true;
    states_cmd_updated_ = true;
}

void GeometricControllerNode::pubRrPrYrTCmd() {
    uav_msgs::AngleRateThrottle cmd;
    Eigen::Vector3d angular_vel;
    double thrust;
    geometricController_.computeControlCmd(thrust, angular_vel);

    cmd.rollRate = angular_vel(0);
    cmd.pitchRate = angular_vel(1);
    cmd.yawRate = angular_vel(2);
    cmd.throttle = thrust / 10;
    control_cmd_pub_.publish(cmd);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_tracking_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    GeometricControllerNode geometricControllerNode(nh, nh_p);
    ros::spin();
    return 0;
}