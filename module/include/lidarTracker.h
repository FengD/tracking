/**
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2018, Feng DING, Hirain.
  *
  *  All rights reserved.
  */

#ifndef _LIDARTRACKER_H_
#define _LIDARTRACKER_H_

#include <string>
#include "dataType.h"

// namespace hirain_itd_ai {

class Tracker;

class LidarTracker {
 public:
	LidarTracker();
	~LidarTracker();
  /**
  * For each frame execute
  */
	void runOneFrame(const DETECTIONS& detections, std::vector<RESULT_DATA>* result);

 private:
	Tracker *tracker_;
};

// } // namespace hirain_itd_ai


#endif // _LIDARTRACKER_H_
