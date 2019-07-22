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

  float px_2 = pow(px, 2);
  float py_2 = pow(py, 2);
  float sum = px_2 + py_2;
  float sq_rt = sqrt(sum);
  float mul = sum * sq_rt;
  // check division by zero

  if (sum < 0.001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << px/sq_rt, py/sq_rt, 0, 0,
    -py/sum, px/sum, 0, 0,
    py*(py*vx - px*vy)/mul,  px*(px*vy - py*vx)/mul, px/sq_rt, py/sq_rt;

  return Hj;
}

VectorXd Tools::CalculatePolar(const VectorXd &x) {
  VectorXd polar = VectorXd(3);
  
  float px = x[0];
  float py = x[1];
  float vx = x[2];
  float vy = x[3];
  float px_2_py_2 = pow(px, 2) + pow(py, 2);
  
  float range = sqrt(px_2_py_2);
  float rho = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / sqrt(px_2_py_2);

  //  if (rho < -pi) {
  //    rho += pi;
  //  } else if (rho > pi) {
  //    rho -= pi;
  //  }
  //

  polar <<
    range,
    rho,
    rho_dot;
  return polar;
}
