#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "dataType.h"

class KalmanFilter {
 public:
  // static const double chi2inv95[10];
  KalmanFilter();
  KAL_DATA initiate(const DETECTBOX& measurement);
  void predict(KAL_MEAN& mean, KAL_COVA& covariance);
  KAL_HDATA project(const KAL_MEAN& mean, const KAL_COVA& covariance);
  KAL_DATA update(const KAL_MEAN& mean, const KAL_COVA& covariance, const DETECTBOX& measurement);

  Eigen::Matrix<float, 1, -1> gating_distance(const KAL_MEAN& mean, const KAL_COVA& covariance, const std::vector<DETECTBOX>& measurements, bool only_position = false);

 private:
  // TODO the dimension needs to be changed
  Eigen::Matrix<float, 8, 8, Eigen::RowMajor> motionMat_;
  Eigen::Matrix<float, 4, 8, Eigen::RowMajor> updateMat_;
  // TODO other weight could be add, and the value should statistic
  float stdWeightPosition_;
  float stdWeightVelocity_;
};

#endif // KALMANFILTER_H
