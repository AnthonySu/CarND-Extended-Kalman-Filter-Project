#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    * use x_k-1|k-1 and state transition equation to compute x_k|k-1
  */
  x_ = F_ * x_;
  MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
    * z is the true measurement
  */

  MatrixXd z_pred = H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose()+R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  //estimate
  x_ = x_ + K_ * (z - z_pred);
  P_ = P_ - K_ * (H_ * P_ * H_.transpose() + R_) * K_.transpose();
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}


int main(int argc, char* argv[]) {
  KalmanFilter KF;
  VectorXd a(2);
  a << 1, 2;
  MatrixXd b(2,2);
  b << 2, 3,
       1, 4;
  KF.Init(a,b,b,b,b,b);
  KF.Predict();
  
  printf("yes\n");
  return 0;
}


