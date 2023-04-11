#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法

int main(int argc, char **argv) {
  srand((unsigned int) time(0));
  int R_theta = rand() % 360;
  Eigen::Vector3d R_axis = Eigen::Vector3d::Random().normalized();
  Eigen::AngleAxisd R_aa(M_PI * R_theta / 180, R_axis);
  Eigen::Matrix3d R = R_aa.toRotationMatrix();
  Eigen::Quaterniond q(R);

  Eigen::Vector3d w(0.01, 0.02, 0.03);

  Sophus::SO3d SO3_R(R);
  Sophus::SO3d SO3_dR = Sophus::SO3d::exp(w);
  Sophus::SO3d R_SO3_updated = SO3_updated.matrix();
  std::cout << "R_SO3_updated:\n" << R_SO3_updated << std::endl;

  double theta = w.norm();
  Eigen::Vector3d w_axis = w / theta;
  Eigen::Matrix3d w_hat;
  w_hat << 0 , -w_axis(2), w_axis(1),
          w_axis(2), 0, -w_axis(0),
          -w_axis(1), w_axis(0), 0;
  Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rodrigues_dR = cos(theta)*I3x3 + (1-cos(theta))*w_axis*w_axis.transpose();
                                + sin(theta)*w_hat;
  Eigen::Matrix3d R_Rodrigues_updated = R * Rodrigues_dR;
  std::cout << "R_Rodrigues_updated:\n" << R_Rodrigues_updated << std::endl;

  Eigen::Quaterniond dq(1, w(0)/2, w(1)/2, w(2)/2);
  Eigen::Quaterniond q_updated1 = (q*dq).normalized();
  dq.normalize();

  Eigen::Quaterniond q_updated2 = q*dq;
  std::cout << "R_Quaterniond_updated1:\n" << q_updated1.toRotationMatrix() << endl;
  std::cout << "R_Quaterniond_updated2:\n" << q_updated2.toRotationMatrix() << endl;

  // Eigen::Matrix3d R_diff = R_SO3_updated - q_updated1.toRotationMatrix();
  // std::cout << "R_diff:\n" << R_diff << std::endl;

  return 0;
}
