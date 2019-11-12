/**
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2018, Feng DING, Hirain.
  *
  *  All rights reserved.
  */

#include "lidarTracker.h"
#include <iostream>

#include "tracker.h"

#define args_max_cosine_distance 0.2

// namespace hirain_itd_ai {

LidarTracker::LidarTracker() {
	tracker_ = new Tracker(args_max_cosine_distance);
}

LidarTracker::~LidarTracker() {
	delete(tracker_);
}

void LidarTracker::runOneFrame(const DETECTIONS& detections, std::vector<RESULT_DATA>* result) {
	tracker_->predict();
	tracker_->update(detections);
	for(Track& track : tracker_->tracks) {
		if(!track.is_confirmed() || track.time_since_update > 1) {
			continue;
		}
		result->push_back(std::make_pair(track.track_id, track.to_tlwh()));
	}
}

// } // namespace hirain_itd_ai
