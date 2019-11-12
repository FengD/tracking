/**
 * Copyright (C) 2019 Hirain
 *
 * License BSD
 *
 * Author: Feng DING
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include "lidarTracker.h"

namespace hirain_itd_ai {

class TrackingNodelet : public nodelet::Nodelet {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle privateNh_;
  ros::NodeHandle mtNh_;
  ros::NodeHandle mtPrivateNh_;
  ros::Subscriber bBoxInputSub_;
  ros::Publisher bBoxOutputPub_;
  std::string bBoxInputTopic_, BoxOutputTopic_;
  int bBoxInputQueue_, bBoxOutputQueue_;
  LidarTracker* lidarTracker_;
  /**
  * bbox msg to Tracking result
  */
  void msgTransform(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& bBox, DETECTIONS *detection) {
    for (int i = 0; i < bBox->boxes.size(); i++) {
      DETECTBOX detectBBox;
      //  TODO ------- fill the missed info
      detectBBox << bBox->boxes[i].pose.position.x,
                    bBox->boxes[i].pose.position.y,
                    bBox->boxes[i].dimensions.x,
                    bBox->boxes[i].dimensions.y;
      detection->push_back(detectBBox);
    }
  }

  /**
  * Tracking result to bbox msg
  */
  void sendTrackingMsg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& bBox, const std::vector<RESULT_DATA> &result) {
    jsk_recognition_msgs::BoundingBoxArray output;
    output.header = bBox->header;
    for (RESULT_DATA r : result) {
      jsk_recognition_msgs::BoundingBox b;
      b.header = bBox->header;
      //  TODO ------- fill the missed info
      b.pose.position.x = r.second(0),
      b.pose.position.y = r.second(1),
      b.pose.position.z = 0,
      b.dimensions.x = r.second(2),
      b.dimensions.y = r.second(3);
      b.dimensions.z = 2,
      output.boxes.push_back(b);
    }
    bBoxOutputPub_.publish(output);
  }

 public:
  TrackingNodelet() {}
  ~TrackingNodelet() {
    delete(lidarTracker_);
  }

  void onInit() {
    ROS_INFO("Init Tracking App");
    nh_ = getNodeHandle();
    privateNh_ = getPrivateNodeHandle();
    mtNh_ = getMTNodeHandle();
    mtPrivateNh_ = getMTPrivateNodeHandle();

    privateNh_.getParam("/Tracking/BBoxInputTopic", bBoxInputTopic_);
    privateNh_.getParam("/Tracking/BBoxOutputTopic", BoxOutputTopic_);
    privateNh_.getParam("/Tracking/BBoxInputQueue", bBoxInputQueue_);
    privateNh_.getParam("/Tracking/BBoxOutputQueue", bBoxOutputQueue_);

    bBoxOutputPub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(BoxOutputTopic_, bBoxOutputQueue_);
    bBoxInputSub_ = nh_.subscribe(bBoxInputTopic_, bBoxInputQueue_, &TrackingNodelet::trackingCallBack, this);

    lidarTracker_ = new LidarTracker;
  }

  void trackingCallBack(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& bBox) {
    // ROS_INFO("objectInput: %d", bBox->boxes.size());
    DETECTIONS detection;
    msgTransform(bBox, &detection);
    std::vector<RESULT_DATA> result;
    lidarTracker_->runOneFrame(detection, &result);
    // ROS_INFO("result: %d", result.size());
    sendTrackingMsg(bBox, result);
  }
}; // TrackingNodelet

} // namespace hirain_itd_ai

PLUGINLIB_EXPORT_CLASS(hirain_itd_ai::TrackingNodelet, nodelet::Nodelet)
