#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159265358979

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_laser_ = R_in;
  Q_ = Q_in;
  
  tools = Tools();
}

void KalmanFilter::Predict() {
  MatrixXd Ft = F_.transpose();
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // compute a Jacobian matrix for the current state vector
  MatrixXd Hj = tools.CalculateJacobian(x_);

  // calculate a predicted measurement
  VectorXd h_x = tools.CalculatePolar(x_);

  // compute error
  VectorXd y = z - h_x;
  // normalize rho error if it is outside the -PI < rho < PI boundaries
  while (y[1] < -PI) {
    y[1] += 2 * PI;
  }
  while (y[1] > PI) {
    y[1] -= 2 * PI;
  }

  MatrixXd Hjt = Hj.transpose();

  MatrixXd S = Hj * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Hjt * Si;
  
  // new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * Hj) * P_;
}
