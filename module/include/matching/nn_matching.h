#ifndef NN_MATCHING_H
#define NN_MATCHING_H

#include <map>
#include "dataType.h"

class NearNeighborDisMetric {
 public:
  enum METRIC_TYPE {
    euclidean = 1,
    cosine
  };

  NearNeighborDisMetric(METRIC_TYPE metric, float matching_threshold);

  DYNAMICM distance(const DETECTBOXSS& features, const std::vector<int> &targets);
  float matching_threshold;

 private:
  typedef Eigen::VectorXf (NearNeighborDisMetric::*PTRFUN)(const DETECTBOXSS&, const DETECTBOXSS&);
  Eigen::VectorXf nncosineDistance(const DETECTBOXSS& x, const DETECTBOXSS& y);
  Eigen::VectorXf nneuclideanDistance(const DETECTBOXSS& x, const DETECTBOXSS& y);

  Eigen::MatrixXf pDist(const DETECTBOXSS& x, const DETECTBOXSS& y);
  Eigen::MatrixXf cosineDistance(const DETECTBOXSS & a, const DETECTBOXSS& b, bool data_is_normalized = false);

  PTRFUN metric;
  std::map<int, DETECTBOXSS> samples;
};

#endif // NN_MATCHING_H
