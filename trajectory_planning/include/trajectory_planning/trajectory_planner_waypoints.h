#ifndef TRAJECTORY_PLANNER_WAYPOINTS_H
#define TRAJECTORY_PLANNER_WAYPOINTS_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace std;

class TrajectoryPlannerWaypoints
{
private:
    
public:
    TrajectoryPlannerWaypoints();
    ~TrajectoryPlannerWaypoints();

    Eigen::MatrixXd minJerkTrajGenerate(
        const Eigen::MatrixXd &path_M,  //3d waypoints coordinates
        const Eigen::MatrixXd &vel_M,   //boundary velocity
        const Eigen::MatrixXd &acc_M,   //boundary acceleration
        const Eigen::VectorXd &time_V   //time allocation in each segment
    );

    //get position,velocity,acceleration in kth segment at t
    Eigen::Vector3d getPosition(Eigen::MatrixXd coeffMatrix, int k, double t);
    Eigen::Vector3d getVelocity(Eigen::MatrixXd coeffMatrix, int k, double t);
    Eigen::Vector3d getAcceleration(Eigen::MatrixXd coeffMatrix, int k, double t);

};








#endif