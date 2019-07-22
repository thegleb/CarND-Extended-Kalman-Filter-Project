#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <cmath>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
   * Cartesian state vector -> polar state vector conversion
   */
  static Eigen::VectorXd CalculatePolar(const Eigen::VectorXd &x);

};

#endif  // TOOLS_H_
