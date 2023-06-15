/**
 * This file is part of VIEO_SLAM
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <mutex>
#include <atomic>
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

namespace VIEO_SLAM {

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer {
 public:
  template <typename _Tp>
  using atomic = std::atomic<_Tp>;

  Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking* pTracking,
         const string& strSettingPath);

  std::atomic<bool> blocalization_mode_ = false;

  // Main thread function. Draw points, keyframes, the current camera pose and the last processed
  // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
  void Run();

  void RequestFinish();

  void RequestStop();

  bool isFinished();

  bool isStopped();

  void Release();

 private:
  bool Stop();

  System* mpSystem;
  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;
  Tracking* mpTracker;

  // 1/fps in ms
  double mT;
  float mImageWidth, mImageHeight;

  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
  int max_cams_num = -1;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  bool mbStopped;
  bool mbStopRequested;
  std::mutex mMutexStop;
};

}  // namespace VIEO_SLAM

#endif  // VIEWER_H
