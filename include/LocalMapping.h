/**
 * This file is part of VIEO_SLAM
 */

#pragma once

#include <mutex>
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "common/macro_creator.h"

namespace VIEO_SLAM {

class IMUInitialization;  // for they includes each other
class Tracking;
class LoopClosing;
class Map;
class KeyFrame;

class LocalMapping {
 public:
  template <typename _Tp, std::size_t _Nm>
  using array = std::array<_Tp, _Nm>;

 public:
  LocalMapping(Map* pMap, const bool bMonocular, const string& strSettingPath);  // should use bool here

  void SetLoopCloser(LoopClosing* pLoopCloser);
  void SetIMUInitiator(IMUInitialization* pIMUInitiator) { mpIMUInitiator = pIMUInitiator; }
  void SetInitLastCamKF(KeyFrame* pKF) {
    if (!mpLastCamKF || !pKF) mpLastCamKF = pKF;
  }

  // Main function
  void Run();

  // mlNewKeyFrames.push_back(pKF) and mbAbortBA=true(stop localBA), if use ,const char state=2: we cannot use
  // Traking::OK/eTrackingState here for Tracking.h and LocalMapping.h include each other
  void InsertKeyFrame(KeyFrame* pKF);

  // Thread Synch
  void RequestStop();   // non-blocking request stop, it will finally be stopped when it's idle, used in localization
                        // mode/CorrectLoop() in LoopClosing thread
  void RequestReset();  // blocking(3ms refreshing) mode
  bool Stop();          // try to stop when requested && allowed to be stopped
  void Release();       // used in bdeactivate_localization_mode_/CorrectLoop() in LoopClosing
  bool isStopped();     // mbStopped
  bool stopRequested();
  bool AcceptKeyFrames();              // if accept KFs, mbAcceptKeyFrames
  void SetAcceptKeyFrames(bool flag);  // mbAcceptKeyFrames=flag
  bool SetNotStop(bool flag);          // true means it cannot be stopped by others

  void InterruptBA();

  void RequestFinish();
  bool isFinished();

  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

  float th_far_pts_ = 0;  // TODO: how to avoid sky feats

 protected:
  bool CheckNewKeyFrames();  // check if New KFs exit (!mlNewKeyFrames.empty())
  // calculate BoW,update mlNewKeyFrames&&mlpRecentAddedMapPoints(RGBD)&&MapPoints' normal&&descriptor, update
  // connections in covisibility graph&& spanning tree, insert KF in mpMap
  void ProcessNewKeyFrame();
  // match CurrentKF with neighbors by BoW && validated by epipolar constraint,triangulate the far/too close points by
  // Linear Triangulation Method/depth data, then check it through positive depth, projection error(chi2 distri.) &&
  // scale consistency,finally update pMP infomation(like mObservations,normal,descriptor,insert in
  // mpMap,KFs,mlpRecentAddedMapPoints)
  void CreateNewMapPoints();

  void MapPointCulling();  // delete some bad && too long ago MapPoints in mlpRecentAddedMapPoints
  // find 2 layers(10,5) of neighbor KFs in covisibility graph, bijection search matches in neighbors and
  // mpCurrentKeyFrame then fuse them,update pMP's normal&&descriptor and CurrentKF's connections in covisibility graph
  void SearchInNeighbors();

  // erase redundant localKFs(all 1st layer covisibility KFs), redundant means 90% close stereo MPs seen by other >=3
  // KFs in same/finer scale
  void KeyFrameCulling();

  cv::Mat SkewSymmetricMatrix(const cv::Mat& v);  // calculate the v^=[0 -c b;c 0 -a;-b a 0]

  bool mbMonocular;

  void ResetIfRequested();
  bool mbResetRequested;
  std::mutex mMutexReset;

  bool CheckFinish();  // mbFinishRequested
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

 private:  // local map (single thread) related params
  // const
  Map* mpMap;
  int mnLocalWindowSize;  // default 10, JW uses 20

  KeyFrame* mpCurrentKeyFrame;

  // for mappoint culling
  std::list<MapPoint*> mlpRecentAddedMapPoints;

  unsigned long mnLastOdomKFId;
  KeyFrame* mpLastCamKF = nullptr;

 private:
  // other multi threads related params
  IMUInitialization* mpIMUInitiator = nullptr;
  LoopClosing* mpLoopCloser;

 protected:  // multi threads related communication params
  bool mbAbortBA = false;

  std::list<KeyFrame*> mlNewKeyFrames;
  std::mutex mMutexNewKFs;

  bool mbAcceptKeyFrames = true;
  std::mutex mMutexAccept;

  bool mbStopped = false;
  bool mbStopRequested = false;
  bool mbNotStop = false;
  std::mutex mMutexStop;

  using ArrayInt2 = array<int, 2>;
  CREATOR_VAR_MULTITHREADS(num_track_inliers, ArrayInt2, , protected, ArrayInt2({0, 100}));
};

}  // namespace VIEO_SLAM
