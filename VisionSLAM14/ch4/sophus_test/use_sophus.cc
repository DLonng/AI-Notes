#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

int main(int argc, char** argv) {
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();  

  Sophus::SO3 SO3_R(R);
  
  Sophus::SO3 SO3_V(0, 0, M_PI / 2);
  
  Eigen::Quaterniond Q(R);
  Sophus::SO3 SO3_Q(Q);

  std::cout << "SO3 from matrix: " << SO3_R << std::endl;
  std::cout << "SO3 from vector: " << SO3_V << std::endl;
  std::cout << "SO3 from quaternion: " << SO3_Q << std::endl;

  Eigen::Vector3d so3 = SO3_R.log();
  std::cout << "so3 = " << so3.transpose() << std::endl;

  std::cout << "so3 hat = \n" << Sophus::SO3::hat(so3) << std::endl;
 
  std::cout << "so3 hat vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)) << std::endl;

  Eigen::Vector3d update_so3(1e-4, 0, 0);
  Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
  std::cout << "SO3 updated = " << SO3_updated << std::endl;

  std::cout << "op SE3" << std::endl;

  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3 SE3_Rt(R, t);
  Sophus::SE3 SE3_Qt(Q, t);

  std::cout << "SE3 from R, t = \n" << SE3_Rt << std::endl;
  std::cout << "SE3 from Q, t = \n" << SE3_Qt << std::endl;

  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();

  std::cout << "se3 = \n" << se3.transpose() << std::endl;

  std::cout << "se3 hat = \n" << Sophus::SE3::hat(se3) << std::endl;
  std::cout << "se3 hat vee = \n" << 
            Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << std::endl;

  Vector6d update_se3;
  update_se3.setZero();
  update_se3(0, 0) = 1e-4d;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;

  std::cout << "SE3 updated = \n" << SE3_updated.matrix() << std::endl;

  return 0;
}





