#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  //std::cout << "Length of ground_truth vector: " << ground_truth.size() << std::endl;
  //std::cout << "Length of estimations vector: " << estimations.size() << std::endl;
  
  //std::cout << "ground_truth vector: " << std::endl;
  //for (auto i: ground_truth)
  //  std::cout << i << ' ';
  
  //std::cout << "estimations vector: " << std::endl;
  //for (auto i: estimations)
  //  std::cout << i << ' ';

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the ground_truth vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()) {
    std::cout << "ERROR: estimation and ground_truth data size are not euqal" << std::endl;
    return rmse;
  }

  if (estimations.size() == 0) {
    std::cout << "ERROR: estimations size is 0" << std::endl;
    return rmse;
  }

  if (ground_truth.size() == 0) {
    std::cout << "ERROR: ground_truth size is 0" << std::endl;
    return rmse;
  }
  
  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  
  if (x_state.size() == 0) {
    std::cout << "ERROR: x_state size is 0" << std::endl;
    return Hj;
  }
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
