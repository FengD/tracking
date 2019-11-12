#include "tracker.h"
#include <iostream>
#include "matching/nn_matching.h"
#include "matching/linear_assignment.h"
#include "kalmanfilter.h"

// namespace hirain_itd_ai {

Tracker::Tracker(float max_cosine_distance, float max_iou_distance, int max_age, int n_init) {
  this->metric = new NearNeighborDisMetric(NearNeighborDisMetric::METRIC_TYPE::euclidean,
                                           max_cosine_distance);
  this->max_iou_distance = max_iou_distance;
  this->max_age = max_age;
  this->n_init = n_init;
  this->kf = new KalmanFilter();
  this->tracks.clear();
  this->next_idx = 1;
}

Tracker::~Tracker() {
  delete metric;
  delete kf;
}

void Tracker::predict() {
  for(Track& track:tracks) {
    track.predit(kf);
  }
}

void Tracker::update(const DETECTIONS &detections) {
  TRACHER_MATCHD res;
  match(detections, res);
  std::vector<MATCH_DATA>& matches = res.matches;

  for(MATCH_DATA& data : matches) {
    int track_idx = data.first;
    int detection_idx = data.second;
    tracks[track_idx].update(this->kf, detections[detection_idx]);
  }

  std::vector<int>& unmatched_tracks = res.unmatched_tracks;

  for(int& track_idx : unmatched_tracks) {
    this->tracks[track_idx].mark_missed();
  }
  std::vector<int>& unmatched_detections = res.unmatched_detections;

  for(int& detection_idx : unmatched_detections) {
    this->initiateTrack(detections[detection_idx]);
  }

  // remove the deleted tracks
  std::vector<Track>::iterator it;
  for(it = tracks.begin(); it != tracks.end();) {
    if((*it).is_deleted()) it = tracks.erase(it);
    else ++it;
  }
}

void Tracker::match(const DETECTIONS &detections, TRACHER_MATCHD &res) {
  std::vector<int> confirmed_tracks;
  std::vector<int> unconfirmed_tracks;
  int idx = 0;
  for(Track& t : tracks) {
    if(t.is_confirmed()) {
      confirmed_tracks.push_back(idx);
    } else {
      unconfirmed_tracks.push_back(idx);
    }
    idx++;
  }
  // Matching Cascade
  TRACHER_MATCHD matchedEgo = linear_assignment::getInstance()->matching_cascade(
    this, &Tracker::gated_matric, this->metric->matching_threshold,
    this->max_age, this->tracks, detections, confirmed_tracks
	);

  std::vector<int> iou_track_candidates;
  iou_track_candidates.assign(unconfirmed_tracks.begin(), unconfirmed_tracks.end());
  std::vector<int>::iterator it;
  for(it = matchedEgo.unmatched_tracks.begin(); it != matchedEgo.unmatched_tracks.end();) {
    int idx = *it;
    if(tracks[idx].time_since_update == 1) { //push into unconfirmed
      iou_track_candidates.push_back(idx);
      it = matchedEgo.unmatched_tracks.erase(it);
      continue;
    }
    ++it;
  }

  // IOU Assignment
  TRACHER_MATCHD matchedIod = linear_assignment::getInstance()->min_cost_matching(
    this, &Tracker::iou_cost,
    this->max_iou_distance,
    this->tracks,
    detections,
    iou_track_candidates,
    matchedEgo.unmatched_detections
	);

  // get result:
  res.matches.assign(matchedEgo.matches.begin(), matchedEgo.matches.end());
  res.matches.insert(res.matches.end(), matchedIod.matches.begin(), matchedIod.matches.end());
  // unmatched_tracks;
  res.unmatched_tracks.assign(matchedEgo.unmatched_tracks.begin(), matchedEgo.unmatched_tracks.end());
  res.unmatched_tracks.insert(res.unmatched_tracks.end(), matchedIod.unmatched_tracks.begin(), matchedIod.unmatched_tracks.end());
  // unmatched detection
  res.unmatched_detections.assign(matchedIod.unmatched_detections.begin(), matchedIod.unmatched_detections.end());
}

void Tracker::initiateTrack(const DETECTBOX &detection) {
  KAL_DATA data = kf->initiate(detection);
  KAL_MEAN mean = data.first;
  KAL_COVA covariance = data.second;

  this->tracks.push_back(Track(mean, covariance, this->next_idx, this->n_init, this->max_age));
  next_idx += 1;
}

DYNAMICM Tracker::gated_matric(
        std::vector<Track> &tracks,
        const DETECTIONS &dets,
        const std::vector<int>& track_indices,
        const std::vector<int>& detection_indices) {
  DETECTBOXSS bboxes(detection_indices.size(), 4);

  int pos = 0;
  for(int i : detection_indices) {
    bboxes.row(pos++) = dets[i];
  }

  std::vector<int> targets;
  for(int i:track_indices) {
    targets.push_back(tracks[i].track_id);
  }

  DYNAMICM cost_matrix = this->metric->distance(bboxes, targets);

  // TODO feature info could be add in the cost mastrix

  return cost_matrix;
}

DYNAMICM Tracker::iou_cost( std::vector<Track> &tracks,
										        const DETECTIONS &dets,
										        const std::vector<int>& track_indices,
										        const std::vector<int>& detection_indices) {
  int rows = track_indices.size();
  int cols = detection_indices.size();
  DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(rows, cols);
  for(int i = 0; i < rows; i++) {
    int track_idx = track_indices[i];
    if(tracks[track_idx].time_since_update > 1) {
      cost_matrix.row(i) = Eigen::RowVectorXf::Constant(cols, INFTY_COST);
      continue;
    }
    DETECTBOX bbox = tracks[track_idx].to_tlwh();
    int csize = detection_indices.size();
    DETECTBOXSS candidates(csize, 4);
    for(int k = 0; k < csize; k++) candidates.row(k) = dets[detection_indices[k]];
    Eigen::RowVectorXf rowV = (1. - iou(bbox, candidates).array()).matrix().transpose();
    cost_matrix.row(i) = rowV;
  }
  return cost_matrix;
}

Eigen::VectorXf Tracker::iou(DETECTBOX& bbox, DETECTBOXSS& candidates) {
  float bbox_tl_1 = bbox[0];
  float bbox_tl_2 = bbox[1];
  float bbox_br_1 = bbox[0] + bbox[2];
  float bbox_br_2 = bbox[1] + bbox[3];
  float area_bbox = bbox[2] * bbox[3];

  Eigen::Matrix<float, -1, 2> candidates_tl;
  Eigen::Matrix<float, -1, 2> candidates_br;
  candidates_tl = candidates.leftCols(2) ;
  candidates_br = candidates.rightCols(2) + candidates_tl;

  int size = int(candidates.rows());

  Eigen::VectorXf res(size);
  for(int i = 0; i < size; i++) {
    float tl_1 = std::max(bbox_tl_1, candidates_tl(i, 0));
    float tl_2 = std::max(bbox_tl_2, candidates_tl(i, 1));
    float br_1 = std::min(bbox_br_1, candidates_br(i, 0));
    float br_2 = std::min(bbox_br_2, candidates_br(i, 1));

    float w = br_1 - tl_1; w = (w < 0? 0: w);
    float h = br_2 - tl_2; h = (h < 0? 0: h);
    float area_intersection = w * h;
    float area_candidates = candidates(i, 2) * candidates(i, 3);
    res[i] = area_intersection/(area_bbox + area_candidates - area_intersection);
  }

  return res;
}

// } // namespace hirain_itd_ai
