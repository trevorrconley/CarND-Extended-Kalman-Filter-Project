#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd F_t_ = F_.transpose();	// we need the transpose matrix of F for our new P
  P_ = F_ * P_ * F_t_ + Q_;	// calculate the new covariance matrix P
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  // this is taken directly from Lesson 5 slide 17
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    if (fabs(x_(0)) < .0001) {
        x_(0) = .0001;
    }
    
  float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float phi = atan2(x_(1),x_(0));
    
  if (rho < 0.0001) {
      rho = 0.0001;
  }
    
  float rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
    
  while (y(1) > M_PI) {
      y(1) -= 2 * M_PI;
  }
  while (y(1) < -M_PI) {
      y(1) += 2 * M_PI;
  }
    
  x_ = x_ + K * y;
    
  // print the output
    std::cout << "K = " << K << std::endl;
  std::cout << "y = " << y << std::endl;
    
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
