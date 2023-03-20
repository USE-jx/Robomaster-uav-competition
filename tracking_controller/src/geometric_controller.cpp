#include "tracking_controller/geometric_controller.h"
#include <iostream>
GeometricController::GeometricController() {}
GeometricController::~GeometricController() {}

//angularRate and Throttle command
void GeometricController::computeControlCmd(double &thrust, Eigen::Vector3d &angularVel) {
    Eigen::Vector3d new_acc;
    computeDesiredAcc(new_acc, odom_pos_, odom_vel_, odom_orient_, desired_pos_, desired_vel_, desired_acc_);
    Eigen::Vector3d attitudeError_v;
    computeAttitudeError(new_acc, odom_orient_, desired_yaw_, attitudeError_v);
    thrust =  -vehicle_mass_ * new_acc.dot(odom_orient_.toRotationMatrix().col(2));
    angularVel = -attitude_gain_.cwiseProduct(attitudeError_v);
                                                                  
}

void GeometricController::computeDesiredAcc(Eigen::Vector3d &new_acc, 
                           Eigen::Vector3d odom_pos,
                           Eigen::Vector3d odom_vel,
                           Eigen::Quaterniond odom_orient,
                           Eigen::Vector3d desired_pos,
                           Eigen::Vector3d desired_vel,
                           Eigen::Vector3d desired_acc) {
    //std::cout << "current position:" << odom_pos << std::endl;
    Eigen::Vector3d position_error = odom_pos - desired_pos;
    //std::cout << "position error:" << position_error.transpose() << std::endl;
    Eigen::Vector3d velocity_error = odom_vel - desired_vel;
    //std::cout << "velocity error:" << velocity_error.transpose() << std::endl;
    Eigen::Vector3d e3(0, 0, 1);
    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
    //std::cout << "e_3:" << e_3.transpose() << std::endl;

    new_acc = (-position_error.cwiseProduct(position_gain_) - velocity_error.cwiseProduct(velocity_gain_))
                / vehicle_mass_ - gravity_ * e3 + desired_acc;
    //std::cout << "desirede acc:" << new_acc.transpose() << std::endl; 
}

void GeometricController::computeAttitudeError(const Eigen::Vector3d new_acc,
                                               Eigen::Quaterniond odom_orient,
                                               double yaw,
                                               Eigen::Vector3d & attitudeError_v) {
    //get current attitude by matrix
    Eigen::Matrix3d currentAttitude = odom_orient.toRotationMatrix();

    //get desired attitude
    Eigen::Vector3d desired_b3 =  -new_acc / new_acc.norm();
    Eigen::Vector3d desired_b1_p (cos(yaw), sin(yaw), 0);
    Eigen::Vector3d desired_b2 = desired_b3.cross(desired_b1_p);
    desired_b2.normalized();
    Eigen::Vector3d desired_b1 = desired_b2.cross(desired_b3);
    Eigen::Matrix3d attitude_matrix;
    attitude_matrix.col(0) = desired_b1;
    attitude_matrix.col(1) = desired_b2;
    attitude_matrix.col(2) = desired_b3;
    Eigen::Vector3d yaw_pitch_roll = attitude_matrix.eulerAngles(0,1,2);
    //ned frame 
    Eigen::Vector3d desiredAttitude;
    desiredAttitude(0) =  yaw_pitch_roll(0);
    desiredAttitude(1) =  yaw_pitch_roll(1);
    desiredAttitude(2) =  yaw_pitch_roll(2);
    //std::cout << "desiredYaw:" << desiredAttitude(2) << std::endl;
    //get attitude error and from matrix to vector
    Eigen::Matrix3d attitudeError_m = 0.5 * (attitude_matrix.transpose() * currentAttitude - currentAttitude.transpose() * attitude_matrix);
    attitudeError_v << attitudeError_m(2, 1), attitudeError_m(0, 2), attitudeError_m(1, 0);
    //std::cout << "attitudeError:" << attitudeError_v.transpose() <<std::endl;
} 