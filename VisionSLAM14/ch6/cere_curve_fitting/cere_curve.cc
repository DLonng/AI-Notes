#include <iostream>
#include <chrono>
#include <vector>

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>


struct CURVE_FITTING_COST {
  CURVE_FITTING_COST (double x, double y) : x_(x), y_(y) {}
    
  template<typename T>
  bool operator() (const T* const abc, T* residual) const {
    
    residual[0] = T(y_)  - ceres::exp(abc[0] * T(x_) * T(x_) + abc[1] * T(x_) + abc[2]);

    return true;
  }



  const double x_;
  const double y_;
};







int main() {
  
  double a = 1.0;
  double b = 2.0;
  double c = 1.0;

  int N = 100;

  double w_sigma = 1.0;

  cv::RNG rng;

  double abc[3] = { 0, 0, 0 };

  std::vector<double> x_data;
  std::vector<double> y_data;

  std::cout << "generating data\n" << std::endl;  

  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
    std::cout << x_data[i] << " " << y_data[i] << std::endl;
  }

  ceres::Problem problem;
  
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
          new CURVE_FITTING_COST(x_data[i], y_data[i])), 
          nullptr, 
          abc
    );
  }


  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  
  ceres::Solve(options, &problem, &summary);
  
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = 
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

  std::cout << "solve time cost = " << time_used.count() << "s" << std::endl;

  std::cout << summary.BriefReport() << std::endl;
  std::cout << "estimated a, b, c = ";
  
  for (auto a : abc) 
    std::cout << a << " ";

  std::cout << std::endl;


  return 0;
}


