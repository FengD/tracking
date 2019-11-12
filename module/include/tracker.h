#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "track.h"
#include <vector>

// namespace hirain_itd_ai {

class NearNeighborDisMetric;
class KalmanFilter;

class Tracker {
 public:
  typedef DYNAMICM (Tracker::* GATED_METRIC_FUNC)(
   std::vector<Track>& tracks,
   const DETECTIONS& dets,
   const std::vector<int>& track_indices,
   const std::vector<int>& detection_indices
  );
  Tracker(float max_cosine_distance, float max_iou_distance = 0.7, int max_age = 30, int n_init = 3);
  ~Tracker();
  void predict();
  void update(const DETECTIONS& detections);

  NearNeighborDisMetric* metric;
  float max_iou_distance;
  int max_age;
  int n_init;
  KalmanFilter* kf;
  int next_idx;
  std::vector<Track> tracks;

 private:
  void match(const DETECTIONS& detections, TRACHER_MATCHD& res);
  void initiateTrack(const DETECTBOX& detection);
  DYNAMICM gated_matric(std::vector<Track>& tracks, const DETECTIONS& dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices);
  // TODO IOU could be used but could be modified
  DYNAMICM iou_cost(    std::vector<Track>& tracks, const DETECTIONS& dets, const std::vector<int>& track_indices, const std::vector<int>& detection_indices);
  // TODO IOU could be used but could be modified
  Eigen::VectorXf iou(DETECTBOX& bbox, DETECTBOXSS &candidates);
};

// } // namespace hirain_itd_ai

#endif // _TRACKER_H_
