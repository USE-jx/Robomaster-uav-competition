#ifndef MINI_JERK_TRAJ_H
#define MINI_JERK_TRAJ_H

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <cmath>

using namespace std;

class TrajectoryGeneratorWaypoints
{
private:
    
public:
    TrajectoryGeneratorWaypoints();
    ~TrajectoryGeneratorWaypoints();

    Eigen::MatrixXd miniJerkTrajGenerate(
        const Eigen::MatrixXd &waypoints,  //3d waypoints coordinates
        const Eigen::MatrixXd &vel,   //boundary velocity
        const Eigen::MatrixXd &acc,   //boundary acceleration
        const Eigen::VectorXd &time   //time allocation in each segment
    );

    //get position,velocity,acceleration in kth segment at t
    Eigen::Vector3d getPosition(Eigen::MatrixXd coeff_matrix, int k, double t);
    Eigen::Vector3d getVelocity(Eigen::MatrixXd coeff_matrix, int k, double t);
    Eigen::Vector3d getAcceleration(Eigen::MatrixXd coeff_matrix, int k, double t);

};


#endif