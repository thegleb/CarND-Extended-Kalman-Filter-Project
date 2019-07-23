#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // state vector
  ekf_.x_ = VectorXd(4);

  // state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
  
  //measurement covariance matrix - laser
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ <<
  0.0225, 0,
  0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;

  // measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;
  
  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
  
  // acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  // process noise covariance
  ekf_.Q_ = MatrixXd(4, 4);

  // identity matrix for reuse
  ekf_.I_ = MatrixXd::Identity(4, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement; fallback to 0, 0
    float px = 0;
    float py = 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // convert radar position to cartesian coordinates and initialize
      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      px = ro * cos(theta);
      py = ro * sin(theta);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // initialize using laser measurement
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    // not enough info to initialize velocity, so we initialize just the px/py
    ekf_.x_ << px, py, 0, 0;

    // cache timestamp so we can get the delta t at the next tick
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // divide by 10000000 to convert to seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // modify the state transition matrix F to update dt
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update the process noise covariance matrix.
  ekf_.Q_ <<  dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
    0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
    dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
    0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  // for computing delta t
  previous_timestamp_ = measurement_pack.timestamp_;

  // prediction step
  ekf_.Predict();
  
  // update step
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // extended kalman filter update for radar
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // regular kalman filter for laser
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
