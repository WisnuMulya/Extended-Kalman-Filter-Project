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

  if (estimations.size()==0) {
    std::cout<<"CalculateRMSE() - Error - estimations size should not be zero";
    return rmse;
  } else if (estimations.size() != ground_truth.size()) {
    std::cout<<"CalculateRMSE() - Error - estimations size should equal ground truth size";
    return rmse;
  }
  
  // accumulate squared residuals
  for (int i=0; i<estimations.size(); ++i) {
    VectorXd res = estimations[i]-ground_truth[i];
    res = res.array()*res.array();
    rmse += res;
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // initialize the Jacobian matrix
  MatrixXd Hj(3,4);
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  if ((px==0) || (py==0)) {
    std::cout << "CalculateJacobian() - Error - Division by Zero";
    return Hj;
  }
  
  // compute the Jacobian matrix
  float px2py2 = pow(px, 2) + pow(py, 2);
  float sqrt_px2py2 = sqrt(px2py2);
  float sqrt_cube_px2py2 = pow(px2py2, 3/2);
  Hj << px/sqrt_px2py2, py/sqrt_px2py2, 0, 0,
        -py/px2py2, px/px2py2, 0, 0,
        py*(vx*py-vy*px)/sqrt_cube_px2py2, px*(vy*px-vx*py)/sqrt_cube_px2py2, px/sqrt_px2py2, py/sqrt_px2py2;

  return Hj;
}
