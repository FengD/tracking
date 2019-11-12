#include "matching/nn_matching.h"
#include <iostream>

NearNeighborDisMetric::NearNeighborDisMetric( NearNeighborDisMetric::METRIC_TYPE metricUse, float matching_threshold) {
  if(metricUse == euclidean) {
    metric = &NearNeighborDisMetric::nneuclideanDistance;
  } else if (metricUse == cosine) {
    metric = &NearNeighborDisMetric::nncosineDistance;
  }
  // else error print error msg of mismatch distance calculation method

  this->matching_threshold = matching_threshold;
  this->samples.clear();
}

DYNAMICM NearNeighborDisMetric::distance(const DETECTBOXSS &features, const std::vector<int>& targets) {
  DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(targets.size(), features.rows());
  // TODO Distance calculate
  // int idx = 0;
  // for(int target : targets) {
  //   cost_matrix.row(idx) = (this->*metric)(this->samples[target], features);
  //   idx++;
  // }
  return cost_matrix;
}

Eigen::VectorXf NearNeighborDisMetric::nncosineDistance( const DETECTBOXSS &x, const DETECTBOXSS &y) {
  Eigen::MatrixXf distances = cosineDistance(x,y);
  Eigen::VectorXf res = distances.colwise().minCoeff().transpose();
  return res;
}

Eigen::VectorXf NearNeighborDisMetric::nneuclideanDistance( const DETECTBOXSS &x, const DETECTBOXSS &y) {
  Eigen::MatrixXf distances = pDist(x,y);
  Eigen::VectorXf res = distances.colwise().maxCoeff().transpose();
  res = res.array().max(Eigen::VectorXf::Zero(res.rows()).array());
  return res;
}

// TODO no weight for each distance
Eigen::MatrixXf NearNeighborDisMetric::pDist(const DETECTBOXSS &x, const DETECTBOXSS &y) {
  int len1 = x.rows(), len2 = y.rows();

  if(len1 == 0 || len2 == 0) {
    return Eigen::MatrixXf::Zero(len1, len2);
  }
  Eigen::MatrixXf res = x * y.transpose()* -2;

  res = res.colwise() + x.rowwise().squaredNorm();
  res = res.rowwise() + y.rowwise().squaredNorm().transpose();
  res = res.array().max(Eigen::MatrixXf::Zero(res.rows(), res.cols()).array());

  return res;
}

Eigen::MatrixXf NearNeighborDisMetric::cosineDistance(const DETECTBOXSS & a, const DETECTBOXSS& b, bool data_is_normalized) {
  if(data_is_normalized == true) {
    //undo:
    assert(false);
  }
  Eigen::MatrixXf res = 1. - (a*b.transpose()).array();
  return res;
}
