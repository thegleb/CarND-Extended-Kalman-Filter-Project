#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  MatrixXd Ft = F_.transpose();
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // standard linear kalman filter stuff

  // compute measurement error (current measurement - prediction)
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
  // calculate a predicted measurement
  VectorXd h_x = tools.CalculatePolar(x_);

  // compute measurement error (current measurement - prediction)
  VectorXd y = z - h_x;

  // normalize rho error if it is outside the -PI < rho < PI boundaries
  // a single if statememnt would've been enough but this feels safer in case -3 * PI < y[1] < 3 * PI
  while (y[1] < -PI) {
    y[1] += 2 * PI;
  }
  while (y[1] > PI) {
    y[1] -= 2 * PI;
  }

  // compute Jacobian matrix for the current state vector
  MatrixXd Hj = tools.CalculateJacobian(x_);
  // and its transposition
  MatrixXd Hjt = Hj.transpose();

  // kalman filter stuff
  MatrixXd S = Hj * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Hjt * Si;
  
  // new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * Hj) * P_;
}
