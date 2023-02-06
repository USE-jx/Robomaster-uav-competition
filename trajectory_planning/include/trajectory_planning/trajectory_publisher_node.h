#ifndef TRAJECTORY_PUBLISHER_NODE_H
#define TRAJECTORY_PUBLISHER_NODE_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <airsim_ros_pkgs/DesiredStates.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

#include "trajectory_planning/trajectory_planner_waypoints.h"

using namespace std;

class TrajectoryPublisherNode
{
private:
    //ros related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher desiredStates_pub_, traj_vis_pub_, cmd_vis_pub_;
    ros::Publisher desiredPose_pub_, currentPose_pub_;
    ros::Subscriber odom_sub_;
    ros::ServiceClient takeoff_client_;
    airsim_ros_pkgs::DesiredStates cmd_;
    nav_msgs::OdometryConstPtr odom_;
    geometry_msgs::PoseStamped desiredPose_, currentPose_;

    //parameters for planner
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Vector3d desiredPos_;
    Eigen::Vector3d dir_;
    Eigen::VectorXd times_;
    Eigen::MatrixXd coeffMatrix_;
    Eigen::MatrixXd waypoints_;
    int waypoint_num_;
    double max_vel_, max_acc_, max_jerk_;
    ros::Time startTime_ ;
    ros::Time finalTime_ ;
    int segment_num_;
    double trajDuration_;

    double startYaw_, finalYaw_;


    TrajectoryPlannerWaypoints * trajPlanWaypoints_ = new TrajectoryPlannerWaypoints();

public:

    TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajectoryPublisherNode();
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &path_M);
    void trajectoryGenerate(const Eigen::MatrixXd &waypoints);
    void desiredStatesPub();
    void displayTrajWithColor();
    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
};








#endif