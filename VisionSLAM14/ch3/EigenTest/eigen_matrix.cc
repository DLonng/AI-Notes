#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main() {
  
  // 2 X 3 float mat
  Eigen::Matrix<float, 2, 3> matrix_23;
  
  // 3 X 1 double mat
  Eigen::Vector3d v_3d;

  // 3 X 3 double mat
  Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();

  // dynamic matrix
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;

  Eigen::MatrixXd matrix_x;

  // op matrix
  matrix_23 << 1, 2, 3, 4, 5, 6;

  std::cout << matrix_23 << std::endl << std::endl;

  for (int i = 0; i < 1; i++) {
    for (int j = 0; j < 2; j++) {
      std::cout << matrix_23(i, j) << std::endl;
    }
  }

  v_3d << 3, 2, 1;

  // matrix_23 must float -> double 
  Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
  std::cout << std::endl << result << std::endl << std::endl;

  // Error!
  // Eigen::Matrix<double, 2, 3> result = matrix_23.cast<double>() * v_3d;

  matrix_33 = Eigen::Matrix3d::Random();
  std::cout << matrix_33 << std::endl << std::endl << std::endl;

  std::cout << matrix_33.transpose() << std::endl << std::endl;
  std::cout << matrix_33.sum() << std::endl << std::endl;
  std::cout << matrix_33.trace() << std::endl << std::endl;
  std::cout << matrix_33.inverse() << std::endl << std::endl;
  std::cout << matrix_33.determinant() << std::endl << std::endl;
  std::cout << 10 * matrix_33 << std::endl << std::endl;


  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);

  std::cout << "Eigen value = " << std::endl << eigen_solver.eigenvalues() << std::endl << std::endl;
  std::cout << "Eigen vector = " << std::endl << eigen_solver.eigenvectors() << std::endl << std::endl;

  Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_nn;
  matrix_nn = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  Eigen::Matrix<double, MATRIX_SIZE, 1> v_nd;

  v_nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

  std::clock_t time_stt = std::clock();

  Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_nn.inverse() * v_nd;
  std::cout << "time use in normal inverse is: " 
    << 1000 * (std::clock() - time_stt) / (double)CLOCKS_PER_SEC 
    << "ms" << std::endl << std::endl;

  time_stt = std::clock();
  x = matrix_nn.colPivHouseholderQr().solve(v_nd);
  std::cout << "time use in Qr composition is: " 
    << 1000 * (std::clock() - time_stt) / (double)CLOCKS_PER_SEC 
    << "ms" << std::endl << std::endl;


  return 0;
}
