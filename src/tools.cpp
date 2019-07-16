#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    std::cout << "The estimation vector must be non-zero and the same size as the ground truth vector.";
    return rmse;
  }

  // TODO: accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // mean
  rmse = rmse / estimations.size();

  // rmse = square root of mean
  return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // check division by zero
  if (px*px + py*py < 0.001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  float px_2 = pow(px, 2);
  float py_2 = pow(py, 2);
  float px_2_py_2 = px_2 + py_2;
  // compute the Jacobian matrix
  Hj << px / sqrt(px_2_py_2), py / sqrt(px_2_py_2), 0, 0,
    -py / (px_2_py_2), px / (px_2_py_2), 0, 0,
    py * (vx * py - vy * px) / pow(px_2_py_2, 3 / 2),  px * (vy * px - vx * py) / pow(px_2_py_2, 3 / 2), px / sqrt(px_2_py_2), py / sqrt(px_2_py_2);

  return Hj;
}
