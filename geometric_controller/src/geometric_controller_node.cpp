#include "geometric_controller/geometric_controller_node.h"

GeometricControllerNode::GeometricControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) 
    : nh_(nh), nh_private_(nh_private), statesCmdUpdated_(false),statesCmdInit_(false) {
    
    AngleRateThrottle_pub_ = 
        nh_.advertise<airsim_ros_pkgs::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame",1);
    AttitudeThrottle_pub_ = 
        nh_.advertise<airsim_ros_pkgs::PoseCmd>("airsim_node/drone_1/pose_cmd_body_frame", 1);
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",1, &GeometricControllerNode::odomCallback,this);
    desiredStates_sub_ =
        nh_.subscribe<airsim_ros_pkgs::DesiredStates>("/reference/desiredStates", 10, &GeometricControllerNode::desiredStatesCallback, this);
    takeoff_client_ = nh_.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client_ = nh_.serviceClient<airsim_ros_pkgs::Land>("/airsim_node/drone_1/land");
    cmd_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &GeometricControllerNode::cmd_pub_timer, this);

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
    if (statesCmdInit_) {
        if (!statesCmdUpdated_) {
            //pubRPYTCmd();
            pubRrPrYrTCmd();
        }
        statesCmdUpdated_ = false;
    }

}

void GeometricControllerNode::desiredStatesCallback(const airsim_ros_pkgs::DesiredStatesConstPtr &msg) {

    desired_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    desired_vel_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    desired_acc_ = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    desired_yaw_ = msg->yaw;
    geometricController_.setDesiredStates(desired_pos_, desired_vel_,desired_acc_,desired_yaw_);
    
    //pubRPYTCmd();
    pubRrPrYrTCmd();
    statesCmdInit_ = true;
    statesCmdUpdated_ = true;
}

void GeometricControllerNode::pubRPYTCmd() {
    airsim_ros_pkgs::PoseCmd cmd;
    Eigen::Vector3d attitude;
    double thrust;
    geometricController_.computeControlCmd2(thrust, attitude);
    cmd.roll = attitude(0);
    cmd.pitch = attitude(1);
    cmd.yaw = attitude(2);
    cmd.throttle = thrust / 10;
    AttitudeThrottle_pub_.publish(cmd);
}
void GeometricControllerNode::pubRrPrYrTCmd() {
    airsim_ros_pkgs::AngleRateThrottle cmd;
    Eigen::Vector3d angularVel;
    double thrust;
    geometricController_.computeControlCmd(thrust, angularVel);

    cmd.rollRate = angularVel(0);
    cmd.pitchRate = angularVel(1);
    cmd.yawRate = angularVel(2);
    cmd.throttle = thrust / 10;
    AngleRateThrottle_pub_.publish(cmd);
}
void GeometricControllerNode::cmd_pub_timer(const ros::TimerEvent &event) {
 

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "geometric_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    GeometricControllerNode geometricControllerNode(nh, nh_p);
    ros::spin();
    return 0;
}