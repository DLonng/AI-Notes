#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>



int main() {

  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

  Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
  std::cout << "rotation_matrix = \n" << rotation_vector.matrix() << std::endl << std::endl;

  rotation_matrix = rotation_vector.toRotationMatrix();

  Eigen::Vector3d v(1, 0, 0);
  Eigen::Vector3d v_rotated = rotation_vector * v;
  std::cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << std::endl << std::endl;

  v_rotated = rotation_matrix * v;
  std::cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << std::endl << std::endl;

  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl << std::endl;

  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.rotate(rotation_vector);
  t.pretranslate(Eigen::Vector3d(1, 3, 4));
  std::cout << "Transform matrix = \n" << t.matrix() << std::endl << std::endl;

  Eigen::Vector3d v_transformed = t * v;
  std::cout << "v transformed = " << v_transformed.transpose() << std::endl << std::endl;


  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
  std::cout << "quaternion = \n" << q.coeffs() << std::endl << std::endl;

  q = Eigen::Quaterniond(rotation_matrix);
  std::cout << "quaternion = \n" << q.coeffs() << std::endl << std::endl;

  v_rotated = q * v;
  std::cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << std::endl << std::endl;


  return 0;
}
