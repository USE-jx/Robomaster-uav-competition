#include "trajectory_planning/trajectory_publisher_node.h"


TrajectoryPublisherNode::TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
:nh_(nh), nh_private_(nh_private)  {
    
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",1, &TrajectoryPublisherNode::odomCallback,this);

    desiredStates_pub_ = 
        nh_.advertise<airsim_ros_pkgs::DesiredStates>("/reference/desiredStates", 50);
    traj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis", 10);
    cmd_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/cmd_vis",10);
    desiredPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/desiredPose", 10);
    currentPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/currentPose", 10);
    takeoff_client_ = nh_.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");    

    //cout << "waypoint init failed!" << endl;
    //get params from sever
    nh_private_.getParam("max_vel", max_vel_);
    nh_private_.getParam("max_acc", max_acc_);
    nh_private_.getParam("waypoint_num", waypoint_num_);
    waypoints_.resize(waypoint_num_, 3);
    cout << "waypoint number:" << waypoint_num_ << endl;

    for (int i = 0; i < waypoint_num_; ++i) {
        nh_private_.getParam("waypoint" + to_string(i) + "/x", waypoints_(i,0));
        nh_private_.getParam("waypoint" + to_string(i) + "/y", waypoints_(i,1));
        nh_private_.getParam("waypoint" + to_string(i) + "/z", waypoints_(i,2));
        
    }
    cout << "waypoints:" << waypoints_ << endl;
    // airsim_ros_pkgs::Takeoff takeoff;
    // takeoff.request.waitOnLastTask = 1;
    // takeoff_client_.call(takeoff);

    trajectoryGenerate(waypoints_);
}


TrajectoryPublisherNode::~TrajectoryPublisherNode() {}

void TrajectoryPublisherNode::trajectoryGenerate(const Eigen::MatrixXd &waypoints) {
    
    //start final vel and acc 
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
     vel(1,0) = 1.3;
     vel(1,1) = -4.67;
     vel(1,2) = 0;
    //  vel(1,0) = max_vel_ * cos(-75.0 / 180.0 * M_PI);
    //  vel(1,1) = max_vel_ * sin(-75.0 / 180.0 * M_PI);
    //  vel(1,2) = 0;

    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    cout << "compute time !" << endl;
    times_ = timeAllocation(waypoints);
    cout << "time vector:" << times_.transpose() << endl;
    coeffMatrix_ = trajPlanWaypoints_->minJerkTrajGenerate(waypoints, vel, acc, times_);
    cout << "trajectory generate finish!" << endl;

    cout << "coeff Matrix:" << coeffMatrix_ << endl;
    finalTime_ = startTime_ = ros::Time::now();
    segment_num_ = times_.size();
    for (int i = 0; i < segment_num_; ++i) {
        finalTime_ += ros::Duration(times_(i));
    }
}

/**
 * compute current pos,vel,acc,yaw to controller
 */
void TrajectoryPublisherNode::desiredStatesPub() {
    cmd_.header.frame_id = odom_->header.frame_id;
    cmd_.header.stamp = odom_->header.stamp;

    trajDuration_ = (finalTime_ - startTime_).toSec();
    //ROS_WARN("timesum:%f:",trajDuration_);
    //cout << "time sum:" << trajDuration_ << endl;
    double t = max(0.0, (odom_->header.stamp - startTime_).toSec());
    //cout <<odom_->header.stamp << startTime_ << endl;
    //cout << "current:" << t << endl;
    
    if (t > trajDuration_) {
        cmd_.position.x = odom_->pose.pose.position.x;
        cmd_.position.y = odom_->pose.pose.position.y;
        cmd_.position.z = odom_->pose.pose.position.z;

        cmd_.velocity.x = 0;
        cmd_.velocity.y = 0;
        cmd_.velocity.z = 0;

        cmd_.acceleration.x = 0;
        cmd_.acceleration.y = 0;
        cmd_.acceleration.z = 0;

    } else {
        for (int i = 0; i < segment_num_; ++i) {
            if (t > times_(i) ) {
                t -= times_(i);
            } else {
                Eigen::Vector3d desiredPos = trajPlanWaypoints_->getPosition(coeffMatrix_, i, t);
                //cout << "desiredPos:" << desiredPos.transpose() << endl;
                //cout << "odom_Pos:" << odom_pos_.transpose() << endl;

                Eigen::Vector3d desiredVel = trajPlanWaypoints_->getVelocity(coeffMatrix_, i, t);
                //cout << "desiredVel:" << desiredVel.transpose() << endl;
                //cout << "odom_vel:" << odom_vel_.transpose() << endl;
                Eigen::Vector3d desiredAcc = trajPlanWaypoints_->getAcceleration(coeffMatrix_, i, t);
                desiredPos_ = desiredPos;
                cmd_.position.x = desiredPos.x();
                cmd_.position.y = desiredPos.y();
                cmd_.position.z = desiredPos.z();
                cmd_.velocity.x = desiredVel.x();
                cmd_.velocity.y = desiredVel.y();
                cmd_.velocity.z = desiredVel.z();
                cmd_.acceleration.x = desiredAcc.x();
                cmd_.acceleration.y = desiredAcc.y();
                cmd_.acceleration.z = desiredAcc.z();
                cmd_.yaw = atan2(cmd_.velocity.y, cmd_.velocity.x);
                //cmd_.yaw = 0.0;
                cout << "desiredyaw:" << cmd_.yaw << endl;
                Eigen::Matrix3d currentAttitude = odom_orient_.toRotationMatrix();
                Eigen::Vector3d RPY = currentAttitude.eulerAngles(0,1,2);
                cout << "currentyaw:" << RPY(0) << endl;
                desiredPose_.header.frame_id = "map";
                desiredPose_.header.stamp = odom_->header.stamp;
                desiredPose_.pose.position.x = desiredPos.x();
                desiredPose_.pose.position.y = desiredPos.y();
                desiredPose_.pose.position.z = desiredPos.z();

                break;
            }
        }
    }
    desiredStates_pub_.publish(cmd_);
    desiredPose_pub_.publish(desiredPose_);

    dir_ << cos(cmd_.yaw), sin(cmd_.yaw), 0;
    drawCmd(desiredPos_, 2 * dir_, 1, Eigen::Vector4d(1, 1, 0, 0.7));

}

void TrajectoryPublisherNode::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_ = msg;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    currentPose_.header.frame_id = "map";
    currentPose_.header.stamp = msg->header.stamp;
    currentPose_.pose.position = msg->pose.pose.position;
    currentPose_.pose.orientation = msg->pose.pose.orientation;
    currentPose_pub_.publish(currentPose_);
    desiredStatesPub();
    displayTrajWithColor();
}

Eigen::VectorXd TrajectoryPublisherNode::timeAllocation(const Eigen::MatrixXd &path_M) {
    Eigen::VectorXd time_V(path_M.rows() - 1);
    for (int i = 0; i < path_M.rows() - 1; ++i) {
        double distance = (path_M.row(i+1) - path_M.row(i)).norm();
        double t = max_vel_ / max_acc_;
        double d = 0.5 * max_acc_ * t * t;
        if (distance < 2 * d) {
            time_V(i) = 2 * sqrt(distance / max_acc_);
        } else {
            time_V(i) = 2 * t + (distance - 2 * d) / max_vel_;
        }
    }
    return time_V;
}

void TrajectoryPublisherNode::displayTrajWithColor() {
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = "map";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  
  points.ns = line_strip.ns = "traj";
  points.id = 0;
  line_strip.id = 1;

  points.action = line_strip.action = visualization_msgs::Marker::ADD;

  points.pose.orientation.w =line_strip.pose.orientation.w = 1.0;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = 0.5;
  points.scale.y = 0.5;

  line_strip.scale.x = 0.3;

  points.color.a = 1.0;
  points.color.g = 1.0;
  line_strip.color.a = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.g = 1.0;

  geometry_msgs::Point pt;
  Eigen::Vector3d pos;
  for (int i = 0; i < segment_num_; ++i) {
      for (double t = 0; t < times_(i); t += 0.01) {
          pos = trajPlanWaypoints_->getPosition(coeffMatrix_, i, t);
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          line_strip.points.push_back(pt);
      }
    }
  for (int i = 0; i < waypoint_num_; ++i) {
      pt.x = waypoints_(i,0);
      pt.y = waypoints_(i,1);
      pt.z = waypoints_(i,2);
      points.points.push_back(pt);
  }
  traj_vis_pub_.publish(points);
  traj_vis_pub_.publish(line_strip);

  ros::Duration(0.001).sleep();
}

void TrajectoryPublisherNode::drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "map";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub_.publish(mk_state);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    TrajectoryPublisherNode trajectoryPublisherNode(nh, nh_p);

    ros::spin();
    return 0;
}