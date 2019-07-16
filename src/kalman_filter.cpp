#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd I = MatrixXd::Identity(2, 2);
VectorXd u = VectorXd(2);

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
  R_ = R_in;
  Q_ = Q_in;
  
  tools = Tools();
}

void KalmanFilter::Predict() {
  MatrixXd Ft = F_.transpose();
  // no motion ... for now
  u << 0, 0;

  x_ = F_ * x_ + u;
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  MatrixXd Hj = tools.CalculateJacobian(x_);

  VectorXd h_x = VectorXd(3);

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float px_2_py_2 = pow(px, 2) + pow(py, 2);

  h_x <<
    sqrt(px_2_py_2),
    atan(py / px),
    (px * vx + py * vy) / sqrt(px_2_py_2);
  
  VectorXd y = z - h_x;
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  
  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * Hj) * P_;
}
