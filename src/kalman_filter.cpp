#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = (F_ * P_ * Ft) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - (H_ * x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S  = (H_ * P_ * Ht) + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // measurement update
  x_ = x_ + (K * y);
  float x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // translate x state to radar measurement space
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho = sqrt(px * px + py * py);
  float theta = atan2(py, px);
  float rho_dot = (px * vx + py * vy)/rho;

  // calculate h_j(x)
  VectorXd z_pred = VectorXd(3);
  z_pred << rho,
            theta,
            rho_dot;
  
  //  VectorXd y = z - z_pred;
  VectorXd y = z - z_pred;
  
  // normalize measurement: phi in y
  double *phi = &y(1);
  *phi = fmod(*phi, 2 * M_PI); // ensure -2pi < phi < 2pi
  // ensure -pi < phi < pi
  if (fabs(*phi) > M_PI) {
    if (*phi > M_PI) {
      *phi = *phi - (2 * M_PI);
    } else {
      *phi = *phi + (2 * M_PI);
    }
  }
   
  MatrixXd Ht = H_.transpose();
  MatrixXd S  = (H_ * P_ * Ht) + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // measurement update
  x_ = x_ + (K * y);
  float x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
