#ifndef TRAJECTORY_REPLAN_H
#define TRAJECTORY_REPLAN_H

#include <ros/ros.h>
//for trajectory planning
#include <uav_msgs/DesiredStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_msgs/Takeoff.h>

//for cv
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include "trajectory_generator/mini_jerk_traj.h"

using namespace std;
using namespace cv;

class TrajectoryReplanNode {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    //for trajectory planning
    ros::Publisher desiredStates_pub_, traj_vis_pub_, cmd_vis_pub_;
    ros::Publisher desiredPose_pub_, currentPose_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    uav_msgs::DesiredStates cmd_;
    nav_msgs::OdometryConstPtr odom_;
    geometry_msgs::PoseStamped desiredPose_, currentPose_;
    ros::ServiceClient takeoff_client_;
    //parameters for trajectory planning
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Vector3d desired_pos_;
    Eigen::Vector3d dir_;
    Eigen::VectorXd times_;
    Eigen::MatrixXd coeff_matrix_;
    Eigen::MatrixXd waypoints_;
    int waypoint_num_;
    double max_vel_, max_acc_, max_jerk_;
    ros::Time start_time_ ;
    ros::Time final_time_ ;
    int segment_num_;
    double traj_duration_;
    double start_yaw_, final_yaw_;
    bool got_circle_flag_;
    int row_idx_ = 1;

    
    //for cv
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber color_image_sub_;
    image_transport::Subscriber depth_image_sub_;
    cv_bridge::CvImageConstPtr color_ptr_, depth_ptr_;
    vector<vector<Point>> pt_;
    


    shared_ptr<TrajectoryGeneratorWaypoints> trajPlanWaypoints_ = make_shared<TrajectoryGeneratorWaypoints>();
public:
    TrajectoryReplanNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajectoryReplanNode();

    inline Eigen::Vector3d transformPixel2World(const Eigen::Vector3d &pixel_and_depth) {
        Eigen::Vector3d point_p (pixel_and_depth(0)*pixel_and_depth(2),
                                 pixel_and_depth(1)*pixel_and_depth(2),
                                 pixel_and_depth(2));
        //cout << "ceicle Center in pixel:" << point_p.transpose() << endl;                         
        Eigen::Matrix3d K;
        K << 160.0, 0.0, 160.0, 0.0, 160.0, 120.0, 0.0, 0.0, 1.0;
        Eigen::Vector3d point_c = K.inverse() * point_p;
        //cout << "ceicle Center in camera:" << point_c.transpose() << endl;

        Eigen::Matrix3d R_c_b;
        R_c_b << 0, 0, 1, 1, 0, 0, 0, 1, 0;
        //cout << R_c_b << endl;
        Eigen::Vector3d t_c_b (0.26, 0, 0);
        Eigen::Vector3d point_b = R_c_b * point_c + t_c_b;
        //cout << "ceicle Center in body:" << point_b.transpose() << endl;
        Eigen::Matrix3d R_b_w = odom_orient_.normalized().toRotationMatrix();
        //Eigen::Matrix3d R_b_w = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_b_w = odom_pos_;
        Eigen::Vector3d point_w = R_b_w * point_b + t_b_w;
        //cout << "uav current pos:" << t_b_w.transpose() << endl;
        // Eigen::Quaterniond q = odom_orient_.normalized();
        // Eigen::Isometry3d T_wb(q);
        // T_wb.pretranslate(odom_pos_);
        // Eigen::Vector3d point_w = T_wb * point_b;
        // cout << "ceicle Center in world:" << point_w.transpose() << endl;
        //cout << "distance to center:" << (point_w - odom_pos_).norm() << endl;
        return point_w;
    }
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &waypoints);
    void trajectoryGenerate(const Eigen::MatrixXd &waypoints);
    void desiredStatesPub();
    void displayTrajWithColor();
    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
    //get depth image and get circle center
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void getCircleCenter(const ros::TimerEvent &e);
};

#endif