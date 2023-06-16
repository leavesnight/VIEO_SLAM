/**
 * This file is part of VIEO_SLAM
 */

#pragma once

#include <mutex>
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "common/multithreadbase.h"
#include "common/macro_creator.h"
#include "common/mlog/log.h"

namespace VIEO_SLAM {

class IMUInitialization;  // for they includes each other
class Tracking;
class LoopClosing;
class Map;
class KeyFrame;

class LocalMapping : public MultiThreadBase {
 public:
  template <typename _Tp>
  using vector = std::vector<_Tp>;
  template <typename _Tp>
  using shared_ptr = std::shared_ptr<_Tp>;
  template <typename _Tp>
  using atomic = std::atomic<_Tp>;

 private:  // local map (single thread) related params
  // const
  Map* mpMap;
  int mnLocalWindowSize;  // default 10, JW uses 20

  KeyFrame* mpCurrentKeyFrame;

  // for mappoint culling
  std::list<MapPoint*> mlpRecentAddedMapPoints;

  // TODO: unify this
  using FrameId = unsigned long;
  FrameId nlast_kfid_odom_ = 0;
  KeyFrame* mpLastCamKF = nullptr;

 private:
  // other multi threads related params
  IMUInitialization* mpIMUInitiator = nullptr;
  LoopClosing* mpLoopCloser;

 protected:  // multi threads related communication special params
  bool mbAbortBA = false;

  // some serial process thread needed input list
  // these caches are for asynchronous Local Mapping thread and tracking thread
  list<KeyFrame*> lnewkeyframes_;
  mutex mutex_newkfs_;

  bool mbAcceptKeyFrames = true;
  std::mutex mMutexAccept;

  // stop lba thread and wait signal
  bool bstopped_ = false;
  bool bstop_requested_ = false;
  mutex mutex_stop_;
  bool bnot_stop_ = false;

  CREATOR_VAR_MULTITHREADS(num_track_inliers, int, , protected, 0);

  void ResetIfRequested();
  void SetFinish() override;
  void ReleaseDynamicMemory(bool block = true);

  bool CheckNewKeyFrames();  // check if New KFs exit (!mlNewKeyFrames.empty())
  // calculate BoW,update mlNewKeyFrames&&mlpRecentAddedMapPoints(RGBD)&&MapPoints' normal&&descriptor, update
  // connections in covisibility graph&& spanning tree, insert KF in mpMap
  void ProcessNewKeyFrame();

  // for SLAM kfs
  void MapPointCulling();  // delete some bad && too long ago MapPoints in mlpRecentAddedMapPoints
  // erase redundant localKFs(all 1st layer covisibility KFs), redundant means 90% close stereo MPs seen by other >=3
  // KFs in same/finer scale
  void KeyFrameCulling();

  // match CurrentKF with neighbors by BoW && validated by epipolar constraint,triangulate the far/too close points by
  // Linear Triangulation Method/depth data, then check it through positive depth, projection error(chi2 distri.) &&
  // scale consistency,finally update pMP infomation(like mObservations,normal,descriptor,insert in
  // mpMap,KFs,mlpRecentAddedMapPoints)
  void CreateNewMapPoints();
  // find 2 layers(10,5) of neighbor KFs in covisibility graph, bijection search matches in neighbors and
  // mpCurrentKeyFrame then fuse them,update pMP's normal&&descriptor and CurrentKF's connections in covisibility graph
  void SearchInNeighbors();

  cv::Mat SkewSymmetricMatrix(const cv::Mat& v);  // calculate the v^=[0 -c b;c 0 -a;-b a 0]

  bool mbMonocular;

 public:
  LocalMapping(Map* pMap, const bool bMonocular, const string& strSettingPath);  // should use bool here
  ~LocalMapping() override;

  void SetLoopCloser(LoopClosing* pLoopCloser);
  void SetIMUInitiator(IMUInitialization* pIMUInitiator) { mpIMUInitiator = pIMUInitiator; }
  void SetInitLastCamKF(KeyFrame* pKF) {
    if (!mpLastCamKF || !pKF) mpLastCamKF = pKF;
  }

  // Main function
  void Run();

  // Thread Synch
  // when new kf added by other threads(e.g. tracking), it couldn't be stopped true means it cannot be stopped by others
  bool SetNotStop(bool flag);
  // non-blocking request stop, it will finally be stopped when it's idle, used in localization mode/CorrectLoop() in
  // LoopClosing thread
  void RequestStop();
  // blocking(3ms refreshing) mode
  void RequestReset(const int8_t id_cam = -1) override;
  bool isStopped();  // mbStopped
  bool stopRequested();
  bool Stop();                         // try to stop when requested && allowed to be stopped
  void Release();                      // used in bdeactivate_localization_mode_/CorrectLoop() in LoopClosing
  bool AcceptKeyFrames();              // if accept KFs, mbAcceptKeyFrames
  void SetAcceptKeyFrames(bool flag);  // mbAcceptKeyFrames=flag
  void InterruptBA();
  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mutex_newkfs_);
    return lnewkeyframes_.size();
  }

  // mlNewKeyFrames.push_back(pKF) and mbAbortBA=true(stop localBA), if use ,const char state=2: we cannot use
  // Traking::OK/eTrackingState here for Tracking.h and LocalMapping.h include each other
  void InsertKeyFrame(const list<KeyFrame *> &pkfs) {
    mbAbortBA = true;  // true will stop localBA
    unique_lock<mutex> lock(mutex_newkfs_);
    lnewkeyframes_.insert(lnewkeyframes_.end(), pkfs.begin(), pkfs.end());
  }

  float th_far_pts_ = 0;  // TODO: how to avoid sky feats
};

}  // namespace VIEO_SLAM
