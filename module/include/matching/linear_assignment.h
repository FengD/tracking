#ifndef LINEAR_ASSIGNMENT_H
#define LINEAR_ASSIGNMENT_H

#include "dataType.h"
#include "tracker.h"

#define INFTY_COST 1e5

class linear_assignment {
 private:
  linear_assignment();
  linear_assignment(const linear_assignment&);
  linear_assignment& operator=(const linear_assignment&);
  static linear_assignment* instance;

 public:
  static linear_assignment* getInstance();
  TRACHER_MATCHD matching_cascade(
    Tracker* distance_metric,
    Tracker::GATED_METRIC_FUNC distance_metric_func,
    float max_distance,
    int cascade_depth,
    std::vector<Track>& tracks,
    const DETECTIONS& detections,
    std::vector<int> &track_indices,
    std::vector<int> detection_indices = std::vector<int>()
  );

  TRACHER_MATCHD min_cost_matching(
    Tracker* distance_metric,
    Tracker::GATED_METRIC_FUNC distance_metric_func,
    float max_distance,
    std::vector<Track>& tracks,
    const DETECTIONS& detections,
    std::vector<int>& track_indices,
    std::vector<int>& detection_indices
  );
};

#endif // LINEAR_ASSIGNMENT_H
