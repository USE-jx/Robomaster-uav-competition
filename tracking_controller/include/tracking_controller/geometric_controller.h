#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>

class GeometricController {
private:
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Vector3d desired_pos_, desired_vel_, desired_acc_;
    double desired_yaw_;
    const double gravity_ = 9.8;

public:
    //parameters sever get params
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    double vehicle_mass_;
     
public:
    GeometricController();
    ~GeometricController();

    //类内默认内联函数
    void setOdometry(const Eigen::Vector3d &odom_pos,
                     const Eigen::Vector3d &odom_vel,
                     const Eigen::Quaterniond &odom_orient) {
        odom_pos_ = odom_pos;
        odom_vel_ = odom_vel;
        odom_orient_ = odom_orient;                 
    }
    void setDesiredStates(const Eigen::Vector3d &desired_pos,
                          const Eigen::Vector3d &desired_vel,
                          const Eigen::Vector3d &desired_acc,
                          double desired_yaw ) {
        desired_pos_ = desired_pos;
        desired_vel_ = desired_vel;
        desired_acc_ = desired_acc;
        desired_yaw_ = desired_yaw;                          
    }
  
    void computeDesiredAcc(Eigen::Vector3d &new_acc, 
                           Eigen::Vector3d odom_pos,
                           Eigen::Vector3d odom_vel,
                           Eigen::Quaterniond odom_orient,
                           Eigen::Vector3d desired_pos,
                           Eigen::Vector3d desired_vel,
                           Eigen::Vector3d desired_acc
                           );
    void computeAttitudeError(const Eigen::Vector3d new_acc,
                              Eigen::Quaterniond odom_orient,
                              double yaw,
                              Eigen::Vector3d & attitudeError_v
                             );
    void computeControlCmd(double &thrust, Eigen::Vector3d &angularVel);                         


};

#endif