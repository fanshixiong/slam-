//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
int main()
{

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }
    
    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    const auto D_rows = 2 * (end_frame_id - start_frame_id);
    // std::cout << D_rows << std::endl;
    Eigen::MatrixXd D;
    D.resize(D_rows, 4);
    for(auto i = start_frame_id; i < end_frame_id; i++){
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw * camera_pose[i].twc;
        // 公式 看ppt
        D.block<1, 3>(2 * (i - start_frame_id), 0) = camera_pose[i].uv[0] * Rcw.block<1, 3>(2, 0) - Rcw.block<1, 3>(0, 0);
        D.block<1, 1>(2 * (i - start_frame_id), 3) = camera_pose[i].uv[0] * tcw.segment(2, 1) - tcw.segment(0, 1);
        D.block<1, 3>(2 * (i - start_frame_id) + 1, 0) = camera_pose[i].uv[1] * Rcw.block<1, 3>(2, 0) - Rcw.block<1, 3>(1, 0);
        D.block<1, 1>(2 * (i - start_frame_id) + 1, 3) = camera_pose[i].uv[1] * tcw.segment(2, 1) - tcw.segment(1, 1);
    }

    // SVD分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D.transpose() * D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector4d lambda = svd.singularValues();
    std::cout << "singularValues: " << lambda.transpose() << std::endl;
    if(lambda(2) / lambda(3) < 1e-3){
        std::cout << "The parallax is not enough" << std::endl;
        return -1;
    }
    Eigen::Vector4d matrixU = svd.matrixU().block<4, 1>(0, 3);
    if(matrixU(3) != 0 && matrixU(2) / matrixU(3) > 0){
        P_est(0) = matrixU(0) / matrixU(3);
        P_est(1) = matrixU(1) / matrixU(3);
        P_est(2) = matrixU(2) / matrixU(3);
    }

    /* your code end */
    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    return 0;
}
