#include "plan_and_control/trajectory_replan_node.h"


TrajectoryReplanNode::TrajectoryReplanNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) 
:nh_(nh), nh_private_(nh_private), got_circle_flag_(false) {
    
    it_ = std::make_unique<image_transport::ImageTransport>(nh_);
    depth_image_sub_ = it_->subscribe("airsim_node/drone_1/front_center/DepthPlanar", 1, 
                        std::bind(&TrajectoryReplanNode::depthImageCallback, this,  std::placeholders::_1));
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",1, &TrajectoryReplanNode::odomCallback,this);
    
    desiredStates_pub_ = 
        nh_.advertise<uav_msgs::DesiredStates>("/reference/desiredStates", 50);
    traj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis", 10);
    cmd_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/cmd_vis",10);
    desiredPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/desiredPose", 10);
    currentPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/currentPose", 10);
    
    timer_ = nh_.createTimer(ros::Duration(0.5), &TrajectoryReplanNode::getCircleCenter, this);
    //cout << "waypoint init failed!" << endl;
    
    takeoff_client_ = nh_.serviceClient<uav_msgs::Takeoff>("/airsim_node/drone_1/takeoff");//get params from sever
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
    // uav_msgs::Takeoff takeoff;
    // takeoff.request.waitOnLastTask = 1;
    // takeoff_client_.call(takeoff);
    
    trajectoryGenerate(waypoints_);
    

}

TrajectoryReplanNode::~TrajectoryReplanNode() {}

void TrajectoryReplanNode::getCircleCenter(const ros::TimerEvent &e) {
    Eigen::Vector3d waypoint(waypoints_(row_idx_,0), waypoints_(row_idx_,1), waypoints_(row_idx_,2));
    double d = (waypoint - odom_pos_).norm();
    //ROS_INFO("dis to goal: %f", d);
    //ROS_INFO("goal : %f %f %f", waypoints_(row_idx_,0), waypoints_(row_idx_,1), waypoints_(row_idx_,2));
    if (d < 2.5) {
        ++row_idx_;
        got_circle_flag_ = false;
        if (row_idx_ == waypoint_num_) {
            --row_idx_;
        }
    }
    //cout << "got_circle:" << got_circle_flag_ << endl;
    
}
void TrajectoryReplanNode::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (got_circle_flag_) return;

    depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    Mat depth = depth_ptr_->image;
    //cout << depth << endl;
    //cout << "rowsandcols:" << depth.rows << " "<< depth.cols << endl;
    Mat gray (depth.rows, depth.cols, CV_8UC1);
    Mat mask = Mat::zeros(depth.size(), CV_8UC1);

    for (int i = 0; i < depth.rows; ++i) {
        for (int j = 0; j < depth.cols; ++j) {
            uchar d = depth.at<float>(i,j) /10 * 255;
            gray.at<uchar>(i,j) = d;
            //cout << depth.at<float>(i,j) << endl; 
        }
    }
    medianBlur(gray, gray, 5);
    vector<Vec3f> circles;
    
    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                50,80,28,15,30);
    //cout << circles.size() << endl;
    if (circles.size() == 1) {
        got_circle_flag_ = true;
        for (int i = 0; i < circles.size(); ++i) {
            Vec3f c = circles[i];
            Point center = Point(c[0],c[1]);      
            //扫描圆环外接矩行的像素深度，把深度不为0的像素深度加和取平均数
            float sum = 0;
            int count = 0;
            for (int v = c[1] - c[2] - 5; v < c[1] + c[2] + 5; ++v) {
                for (int u = c[0] - c[2] - 5; u < c[0] + c[2] + 5; ++u) {
                    float d = depth.ptr<float>(v)[u];
                    if (d == 0) continue;
                    sum += d;
                    ++count;
                }
            }
            double depth5 = sum / count;
            //cout << "depth:" << depth5 << endl;
            cout << "row_index:" << row_idx_ << endl;
            //cout << "depth center:" << center << endl;
            Eigen::Vector3d pixel_and_depth(c[0], c[1], depth5);
            Eigen::Vector3d point_w = transformPixel2World(pixel_and_depth); 
            double distance = (point_w - waypoints_.row(row_idx_).transpose()).norm();
            cout << "distance:" << distance << endl;
            if (distance > 0.5) {
                Eigen::MatrixXd replan_matrix = Eigen::MatrixXd::Zero(8 - row_idx_ + 1, 3);
                replan_matrix(0,0) = odom_pos_(0);
                replan_matrix(0,1) = odom_pos_(1);
                replan_matrix(0,2) = odom_pos_(2);
                replan_matrix(1,0) = point_w(0);
                replan_matrix(1,1) = point_w(1);
                replan_matrix(1,2) = point_w(2);
                replan_matrix.bottomRows(8 - row_idx_ - 1) = waypoints_.bottomRows(8 - row_idx_ - 1);
                cout << "replan_matrix:" << replan_matrix << endl;
                       
                trajectoryGenerate(replan_matrix);
            }
            
            circle(gray,center, 1, Scalar(255,255,255),1,LINE_AA);
            int radius = c[2];
            circle(gray, center, radius, Scalar(255,255,255),5,LINE_AA);
        }
    }
    imshow("depth2gray", gray);
    cv::waitKey(1);
}

void TrajectoryReplanNode::trajectoryGenerate(const Eigen::MatrixXd &waypoints) {
    
    //start final vel and acc 
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
    vel(0,0) = odom_vel_(0);
    vel(0,1) = odom_vel_(1);
    vel(0,2) = odom_vel_(2);
    vel(1,0) = 0.8;
    vel(1,1) = -4.67;
    vel(1,2) = 0;
    

    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    
    cout << "compute time !" << endl;
    times_ = timeAllocation(waypoints);
    cout << "time vector:" << times_.transpose() << endl;
    coeff_matrix_ = trajPlanWaypoints_->miniJerkTrajGenerate(waypoints, vel, acc, times_);
    cout << "trajectory generate finish!" << endl;

    final_time_ = start_time_ = ros::Time::now();
    segment_num_ = times_.size();
    for (int i = 0; i < segment_num_; ++i) {
        final_time_ += ros::Duration(times_(i));
    }
}

/**
 * compute current pos,vel,acc,yaw to controller
 */
void TrajectoryReplanNode::desiredStatesPub() {
    cmd_.header = odom_->header;
    
    traj_duration_ = (final_time_ - start_time_).toSec();
    //ROS_WARN("timesum:%f:",trajDuration_);
    //cout << "time sum:" << trajDuration_ << endl;
    double t = max(0.0, (odom_->header.stamp - start_time_).toSec());
    //cout <<odom_->header.stamp << startTime_ << endl;
    //cout << "current:" << t << endl;
    
    if (t > traj_duration_) {
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
                Eigen::Vector3d desiredPos = trajPlanWaypoints_->getPosition(coeff_matrix_, i, t);
                //cout << "desiredPos:" << desiredPos.transpose() << endl;
                //cout << "odom_Pos:" << odom_pos_.transpose() << endl;

                Eigen::Vector3d desiredVel = trajPlanWaypoints_->getVelocity(coeff_matrix_, i, t);
                //cout << "desiredVel:" << desiredVel.transpose() << endl;
                //cout << "odom_vel:" << odom_vel_.transpose() << endl;
                Eigen::Vector3d desiredAcc = trajPlanWaypoints_->getAcceleration(coeff_matrix_, i, t);
                desired_pos_ = desiredPos;
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
                //cout << "desiredyaw:" << cmd_.yaw << endl;
                Eigen::Matrix3d currentAttitude = odom_orient_.toRotationMatrix();
                Eigen::Vector3d RPY = currentAttitude.eulerAngles(0,1,2);
                //cout << "currentyaw:" << RPY(0) << endl;
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
    drawCmd(desired_pos_, 2 * dir_, 1, Eigen::Vector4d(1, 1, 0, 0.7));

}

void TrajectoryReplanNode::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
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

Eigen::VectorXd TrajectoryReplanNode::timeAllocation(const Eigen::MatrixXd &waypoints) {
    Eigen::VectorXd time(waypoints.rows() - 1);
    double current_vel = odom_vel_.norm();
    
        for (int i = 0; i < waypoints.rows() - 1; ++i) {
            double distance = (waypoints.row(i+1) - waypoints.row(i)).norm();
            double t = max_vel_ / max_acc_;
            double d = 0.5 * max_acc_ * t * t;
            if (distance < 2 * d) {
                time(i) = 2 * sqrt(distance / max_acc_);
            } else {
                time(i) = 2 * t + (distance - 2 * d) / max_vel_;
            }
            
        }
        if (current_vel != 0) {
            double distance = (waypoints.row(1) - waypoints.row(0)).norm();
            time(0) = distance  / current_vel;
            for (int i = 1; i < waypoints.rows() - 1; ++i) {
                double distance = (waypoints.row(i+1) - waypoints.row(i)).norm();
                time(i) = distance * 3.4 / max_vel_;
            }
        }

    return time;
}


void TrajectoryReplanNode::displayTrajWithColor() {
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
          pos = trajPlanWaypoints_->getPosition(coeff_matrix_, i, t);
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

void TrajectoryReplanNode::drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_replan_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    namedWindow("depth2gray");
  
    TrajectoryReplanNode trajectory_replan_node(nh, nh_private);

    ros::spin();
    return 0;
}