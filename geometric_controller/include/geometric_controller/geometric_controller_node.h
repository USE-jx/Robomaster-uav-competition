#ifndef GEOMETRIC_CONTROLLER_NODE_H
#define GEOMETRIC_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <airsim_ros_pkgs/AngleRateThrottle.h>
#include <airsim_ros_pkgs/PoseCmd.h>
#include <airsim_ros_pkgs/DesiredStates.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "geometric_controller/geometric_controller.h"

using namespace std;

class GeometricControllerNode {
private:
    //ros related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher AngleRateThrottle_pub_, AttitudeThrottle_pub_;
    ros::Subscriber odom_sub_, desiredStates_sub_;

    ros::ServiceClient takeoff_client_, land_client_;

    ros::Timer cmd_pub_timer_;

    //parameters for controller
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d desired_pos_, desired_vel_, desired_acc_;
    Eigen::Quaterniond desired_orient_; 
    double desired_yaw_;

    GeometricController geometricController_;
    bool takeoffFlag = 0;
    bool statesCmdUpdated_, statesCmdInit_;

public:
    GeometricControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~GeometricControllerNode();
    
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void desiredStatesCallback(const airsim_ros_pkgs::DesiredStatesConstPtr &msg);
    void pubRPYTCmd();
    void pubRrPrYrTCmd();
    void cmd_pub_timer(const ros::TimerEvent &event);
};

#endif