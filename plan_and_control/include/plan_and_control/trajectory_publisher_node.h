#ifndef TRAJECTORY_PUBLISHER_NODE_H
#define TRAJECTORY_PUBLISHER_NODE_H

//ros and related msg
#include <ros/ros.h>
#include <uav_msgs/DesiredStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

//other utils
#include <vector>
#include <memory>
#include <Eigen/Eigen>

#include "trajectory_generator/mini_jerk_traj.h"

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
    uav_msgs::DesiredStates cmd_;
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

    shared_ptr<TrajectoryGeneratorWaypoints> trajPlanWaypoints_ = make_shared<TrajectoryGeneratorWaypoints>();

public:

    TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajectoryPublisherNode();
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &waypoints);
    void trajectoryGenerate(const Eigen::MatrixXd &waypoints);
    void desiredStatesPub();

    void displayTrajWithColor();
    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
};

#endif