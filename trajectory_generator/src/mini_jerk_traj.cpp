#include "trajectory_generator/mini_jerk_traj.h"

TrajectoryGeneratorWaypoints::TrajectoryGeneratorWaypoints() {}

TrajectoryGeneratorWaypoints::~TrajectoryGeneratorWaypoints() {}


Eigen::MatrixXd TrajectoryGeneratorWaypoints::miniJerkTrajGenerate(
        const Eigen::MatrixXd &waypoints,  //3d waypoints coordinates nx3
        const Eigen::MatrixXd &vel,   //boundary velocity 2x3
        const Eigen::MatrixXd &acc,   //boundary acceleration 2x3
        const Eigen::VectorXd &time ) { //time allocation in each segment
    //固定求jerk可以不写，用qp求不确定是jerk和snap时候可以写
    //int p_order = 2 * d_order -1;  //the order of polynomial  (5)
    //int p_num1d = p_order + 1;   //the number of coeff in each segment  (c0 ~ c5)
    int segments = time.size();  //the number of segments

    // M * coeffMatrix = b  就能解出系数矩阵
    Eigen::MatrixXd coeffMatrix; 
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6 * segments, 6 * segments);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6 * segments, 3);

    /**
     * 构造M和b矩阵
     * 1 起始pva 和 末尾pva约束
     * 2 中间点p v a jerk snap 连续约束，即第一段T = 第二段0
     * 这两个约束正好填满M矩阵
     */
    //中间点连续需要用到T到T^5
    Eigen::VectorXd T1 = time;
    Eigen::VectorXd T2 = T1.cwiseProduct(T1);
    Eigen::VectorXd T3 = T2.cwiseProduct(T1);
    Eigen::VectorXd T4 = T3.cwiseProduct(T1);
    Eigen::VectorXd T5 = T4.cwiseProduct(T1);

    //initial pva
    M(0, 0) = 1;
    M(1, 1) = 1;
    M(2, 2) = 2;

    b.row(0) = waypoints.row(0);
    b.row(1) = vel.row(0);
    b.row(2) = acc.row(0);

    //p v a jerk snap 连续
    for (int i = 0; i < segments-1; ++i) {
        
        //由前一段的时间得到下一点的开始位置
        M(6 * i + 3, 6 * i) = 1;
        M(6 * i + 3, 6 * i + 1) = T1(i);
        M(6 * i + 3, 6 * i + 2) = T2(i);
        M(6 * i + 3, 6 * i + 3) = T3(i);
        M(6 * i + 3, 6 * i + 4) = T4(i);
        M(6 * i + 3, 6 * i + 5) = T5(i);

        b.row(6 * i + 3) = waypoints.row(i + 1);
        
        //位置连续，前一段T时刻与后一段0时刻
        M(6 * i + 4, 6 * i) = 1;
        M(6 * i + 4, 6 * i + 1) = T1(i);
        M(6 * i + 4, 6 * i + 2) = T2(i);
        M(6 * i + 4, 6 * i + 3) = T3(i);
        M(6 * i + 4, 6 * i + 4) = T4(i);
        M(6 * i + 4, 6 * i + 5) = T5(i);
        M(6 * i + 4, 6 * i + 6) = -1;

        //速度连续
        M(6 * i + 5, 6 * i + 1) = 1;
        M(6 * i + 5, 6 * i + 2) = 2 * T1(i);
        M(6 * i + 5, 6 * i + 3) = 3 * T2(i);
        M(6 * i + 5, 6 * i + 4) = 4 * T3(i);
        M(6 * i + 5, 6 * i + 5) = 5 * T4(i);
        M(6 * i + 5, 6 * i + 7) = -1;     

        //加速度连续
        M(6 * i + 6, 6 * i + 2) = 2;
        M(6 * i + 6, 6 * i + 3) = 6 * T1(i);
        M(6 * i + 6, 6 * i + 4) = 12 * T2(i);
        M(6 * i + 6, 6 * i + 5) = 20 * T3(i);
        M(6 * i + 6, 6 * i + 8) = -2;    

        //jerk连续
        M(6 * i + 7, 6 * i + 3) = 6;
        M(6 * i + 7, 6 * i + 4) = 24 * T1(i);
        M(6 * i + 7, 6 * i + 5) = 60 * T2(i);
        M(6 * i + 7, 6 * i + 9) = -6;    

        //snap连续
        M(6 * i + 8, 6 * i + 4) = 24;
        M(6 * i + 8, 6 * i + 5) = 120 * T1(i);
        M(6 * i + 8, 6 * i + 10) = -24; 
    }

    //末端pva
    M(6 * segments - 3, 6 * segments - 6) = 1;
    M(6 * segments - 3, 6 * segments - 5) = T1(segments-1);
    M(6 * segments - 3, 6 * segments - 4) = T2(segments-1);
    M(6 * segments - 3, 6 * segments - 3) = T3(segments-1);
    M(6 * segments - 3, 6 * segments - 2) = T4(segments-1);
    M(6 * segments - 3, 6 * segments - 1) = T5(segments-1);

    M(6 * segments - 2, 6 * segments - 5) = 1;
    M(6 * segments - 2, 6 * segments - 4) = 2 * T1(segments-1);
    M(6 * segments - 2, 6 * segments - 3) = 3 * T2(segments-1);
    M(6 * segments - 2, 6 * segments - 2) = 4 * T3(segments-1);
    M(6 * segments - 2, 6 * segments - 1) = 5 * T4(segments-1);

    M(6 * segments - 1, 6 * segments - 4) = 2 ; 
    M(6 * segments - 1, 6 * segments - 3) = 6 * T1(segments-1); 
    M(6 * segments - 1, 6 * segments - 2) = 12 * T2(segments-1); 
    M(6 * segments - 1, 6 * segments - 1) = 20 * T3(segments-1); 

    b.row(6 * segments - 3) = waypoints.row(segments);
    b.row(6 * segments - 2) = vel.row(1);
    b.row(6 * segments - 1) = acc.row(1);

    coeffMatrix = M.fullPivLu().solve(b); 
    //transfrom to segments x (6*3) size 每行是一段x,y,z的系数
    Eigen::MatrixXd coeffMatrix_t(segments, 6*3);
    for (int i = 0; i < segments; ++i) {
        coeffMatrix_t.block(i, 0, 1, 6) = coeffMatrix.col(0).segment(6*i, 6).transpose();
        coeffMatrix_t.block(i, 6, 1, 6) = coeffMatrix.col(1).segment(6*i, 6).transpose();
        coeffMatrix_t.block(i, 12, 1, 6) = coeffMatrix.col(2).segment(6*i, 6).transpose();
    }
    return coeffMatrix_t;

}     

    
//get position,velocity,acceleration in kth segment at t
//p = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
Eigen::Vector3d TrajectoryGeneratorWaypoints::getPosition(Eigen::MatrixXd coeff_matrix, int k, double t) {
    Eigen::Vector3d position(0.0, 0.0, 0.0);
    for (int dim = 0; dim < 3; ++dim) {
        double tn = 1.0;
        for (int i = 0; i < 6; ++i) {
            position(dim) += coeff_matrix.row(k).segment(dim*6,6)(i) * tn;
            tn *= t;
        }//这样写不太好，可能写成矩阵运算更快
    }
    return position;
}
//v = c1 + 2*c2*t + 3c3*t^2 + 4c4*t^3 + 5c5*t^4
Eigen::Vector3d TrajectoryGeneratorWaypoints::getVelocity(Eigen::MatrixXd coeff_matrix, int k, double t) {
    Eigen::Vector3d velocity(0.0, 0.0, 0.0);
    for (int dim = 0; dim < 3; ++dim) {
        double tn = 1.0;
        for (int i = 1; i < 6; ++i) {
            velocity(dim) += coeff_matrix.row(k).segment(dim*6, 6)(i) * i * tn;
            tn *= t;
        }
    }
    return velocity;
}
//a = 2c2 + 6c3*t + 12c4*t^2 + 20c5*t^3
Eigen::Vector3d TrajectoryGeneratorWaypoints::getAcceleration(Eigen::MatrixXd coeff_matrix, int k, double t) {
    Eigen::Vector3d acceleration(0.0, 0.0, 0.0);
    for (int dim = 0; dim < 3; ++dim) {
        double tn = 1.0;
        for (int i = 2; i < 6; ++i) {
            acceleration(dim) += coeff_matrix.row(k).segment(dim*6, 6)(i) * i * (i - 1) * tn;
            tn *= t; 
        }
    }
    return acceleration;
}