/**
 * This file is part of VIEO_SLAM
 */

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "IMUInitialization.h"  //zzh

#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"

#include <mutex>

namespace VIEO_SLAM {

class IMUInitialization;  // zzh, for they includes each other

class Tracking;
class LoopClosing;
class Map;
class KeyFrame;

class LocalMapping {
  unsigned long mnLastOdomKFId;
  KeyFrame* mpLastCamKF;

  // Local Window size
  int mnLocalWindowSize;  // default 10, JW uses 20

  // created by zzh over.

 public:
  LocalMapping(Map* pMap, const bool bMonocular, const string& strSettingPath);  // should use bool here

  void SetLoopCloser(LoopClosing* pLoopCloser);
  void SetTracker(Tracking* pTracker);
  void SetIMUInitiator(IMUInitialization* pIMUInitiator) { mpIMUInitiator = pIMUInitiator; }  // zzh

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
  void Release();       // used in mbDeactivateLocalizationMode/CorrectLoop() in LoopClosing
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

  Map* mpMap;

  LoopClosing* mpLoopCloser;
  IMUInitialization* mpIMUInitiator;  // zzh
  Tracking* mpTracker;                // unused

  std::list<KeyFrame*> mlNewKeyFrames;
  // std::list<Tracking::eTrackingState> mlNewKFStates;

  KeyFrame* mpCurrentKeyFrame;

  std::list<MapPoint*> mlpRecentAddedMapPoints;

  std::mutex mMutexNewKFs;

  bool mbAbortBA;

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;
};

}  // namespace VIEO_SLAM

#endif  // LOCALMAPPING_H
