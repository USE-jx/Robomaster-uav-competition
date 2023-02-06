#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>

class GeometricController {
private:
    bool initialized_parameters_;
    Eigen::Vector3d odom_pos_;
    Eigen::Vector3d odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Vector3d desired_pos_;
    Eigen::Vector3d desired_vel_;
    Eigen::Vector3d desired_acc_;
    double desired_yaw_;

public:
    GeometricController();
    ~GeometricController();

    void setOdometry(const Eigen::Vector3d &odom_pos,
                     const Eigen::Vector3d &odom_vel,
                     const Eigen::Quaterniond &odom_orient);
    void setDesiredStates(const Eigen::Vector3d &desired_pos,
                          const Eigen::Vector3d &desired_vel,
                          const Eigen::Vector3d &desired_acc,
                          double desired_yaw );
    void computeControlCmd(double &thrust, Eigen::Vector3d &angularVel);
    void computeControlCmd2(double &thrust, Eigen::Vector3d &attitude);
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
    void computeDesiredAttitude(const Eigen::Vector3d new_acc,
                              Eigen::Quaterniond odom_orient,
                              double yaw,
                              Eigen::Vector3d & attitude);
    //parameters sever get params
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    double vehicle_mass_;
    double gravity_ = 9.8; 
};




#endif