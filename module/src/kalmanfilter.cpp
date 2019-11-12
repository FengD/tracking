#include "kalmanfilter.h"
#include <Eigen/Cholesky>

// // TODO feature map
// const double KalmanFilter::chi2inv95[10] = {
//     0,
//     3.8415,
//     5.9915,
//     7.8147,
//     9.4877,
//     11.070,
//     12.592,
//     14.067,
//     15.507,
//     16.919
// };
KalmanFilter::KalmanFilter() {
  // TODO dimension
  int ndim = 4;
  double dt = 1.;
  // TODO dimension
  motionMat_ = Eigen::MatrixXf::Identity(8, 8);
  for(int i = 0; i < ndim; i++) {
      motionMat_(i, ndim+i) = dt;
  }
  updateMat_ = Eigen::MatrixXf::Identity(4, 8);

  // TODO the value may change
  this->stdWeightPosition_ = 1. / 20;
  this->stdWeightVelocity_ = 1. / 160;
}

KAL_DATA KalmanFilter::initiate(const DETECTBOX &measurement) {
  DETECTBOX mean_pos = measurement;
  DETECTBOX mean_vel;
  // TODO dimension
  for(int i = 0; i < 4; i++)
    mean_vel(i) = 0;

  KAL_MEAN mean;
  for(int i = 0; i < 8; i++){
    if(i < 4)
      mean(i) = mean_pos(i);
    else
      mean(i) = mean_vel(i - 4);
  }

  // TODO
  KAL_MEAN std;
  std(0) = 2 * stdWeightPosition_ * measurement[3];
  std(1) = 2 * stdWeightPosition_ * measurement[3];
  std(2) = 1e-2;
  std(3) = 2 * stdWeightPosition_ * measurement[3];
  std(4) = 10 * stdWeightVelocity_ * measurement[3];
  std(5) = 10 * stdWeightVelocity_ * measurement[3];
  std(6) = 1e-5;
  std(7) = 10 * stdWeightVelocity_ * measurement[3];

  KAL_MEAN tmp = std.array().square();
  KAL_COVA var = tmp.asDiagonal();
  return std::make_pair(mean, var);
}

void KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance) {
  //revise the data;
  // TODO dimension
  DETECTBOX std_pos;
  std_pos << stdWeightPosition_ * mean(3),
          stdWeightPosition_ * mean(3),
          1e-2,
          stdWeightPosition_ * mean(3);
  // TODO dimension
  DETECTBOX std_vel;
  std_vel << stdWeightVelocity_ * mean(3),
          stdWeightVelocity_ * mean(3),
          1e-5,
          stdWeightVelocity_ * mean(3);
  KAL_MEAN tmp;
  // TODO dimension
  tmp.block<1,4>(0,0) = std_pos;
  tmp.block<1,4>(0,4) = std_vel;
  tmp = tmp.array().square();
  KAL_COVA motion_cov = tmp.asDiagonal();
  KAL_MEAN mean1 = this->motionMat_ * mean.transpose();
  KAL_COVA covariance1 = this->motionMat_ * covariance *(motionMat_.transpose());
  covariance1 += motion_cov;

  mean = mean1;
  covariance = covariance1;
}

KAL_HDATA KalmanFilter::project(const KAL_MEAN &mean, const KAL_COVA &covariance) {
  DETECTBOX std;
  std << stdWeightPosition_ * mean(3), stdWeightPosition_ * mean(3),
          1e-1, stdWeightPosition_ * mean(3);
  KAL_HMEAN mean1 = updateMat_ * mean.transpose();
  KAL_HCOVA covariance1 = updateMat_ * covariance * (updateMat_.transpose());
  Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
  diag = diag.array().square().matrix();
  covariance1 += diag;

  return std::make_pair(mean1, covariance1);
}

KAL_DATA KalmanFilter::update( const KAL_MEAN &mean,
                               const KAL_COVA &covariance,
                               const DETECTBOX &measurement) {
  KAL_HDATA pa = project(mean, covariance);
  KAL_HMEAN projected_mean = pa.first;
  KAL_HCOVA projected_cov = pa.second;

  // TODO dimension
  Eigen::Matrix<float, 4, 8> B = (covariance * (updateMat_.transpose())).transpose();
  Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
  Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean; //eg.1x4
  auto tmp = innovation*(kalman_gain.transpose());
  KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();
  KAL_COVA new_covariance = covariance - kalman_gain*projected_cov*(kalman_gain.transpose());
  return std::make_pair(new_mean, new_covariance);
}

Eigen::Matrix<float, 1, -1> KalmanFilter::gating_distance(
  const KAL_MEAN &mean,
  const KAL_COVA &covariance,
  const std::vector<DETECTBOX> &measurements,
  bool only_position) {
  KAL_HDATA pa = this->project(mean, covariance);
  if(only_position) {
    printf("not implement!");
    exit(0);
  }
  KAL_HMEAN mean1 = pa.first;
  KAL_HCOVA covariance1 = pa.second;

  // TODO dimension
  DETECTBOXSS d(measurements.size(), 4);
  int pos = 0;
  for(DETECTBOX box:measurements) {
    d.row(pos++) = box - mean1;
  }
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
  Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
  auto zz = ((z.array())*(z.array())).matrix();
  auto square_maha = zz.colwise().sum();
  return square_maha;
}
