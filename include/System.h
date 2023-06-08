/**
 * This file is part of VIEO_SLAM
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "IMUInitialization.h"
// created by zzh

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "common/mlog/log.h"

namespace VIEO_SLAM {
class IMUInitialization;  // zzh

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System {
  // used to make pcl
  cv::FileStorage fsSettings;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  IMUInitialization* mpIMUInitiator;
  // System thread: a new IMUInitialization thread added
  std::thread* mptIMUInitialization;

 public:
  enum eOdom { ENCODER = 0, IMU, BOTH };

  // Process the given (IMU/encoder)odometry data. mode==0:Encoder data 2 vl,vr; 1:qIMU data 4 qxyzw; 2:Both 6
  // vl,vr,qxyzw; 3:Pure-IMU data 6 ax~z,wx~z(opposite of the order of EuRoc)
  cv::Mat TrackOdom(const double& timestamp, const double* odomdata, const char mode);
  // please call this after Shutdown(), Full BA (column/at the end of execution) in V-B of the VIORBSLAM paper
  void FinalGBA(int nIterations = 15, bool bRobust = false);

  // TODO: Save/Load functions
  // we will save filename(like "KeyFrameTrajectoryIMU.txt")(including t,q,v,bg,ba) from Tcw by using Tbc or directly
  // from Twb
  void SaveKeyFrameTrajectoryNavState(const string& filename, bool bUseTbc = true);
  void SaveTrajectoryNavState(const string& filename, bool bUseTbc = true);
  void SaveMap(const string& filename, bool bPCL = true, bool bUseTbc = true, bool bSaveBadKF = false);
  bool LoadMap(const string& filename, bool bPCL = true,
               bool bReadBadKF = false);  // if read bad KFs, we correct mpTracker->mlpReferences
  void SaveFrame(string foldername, const cv::Mat& im, const cv::Mat& depthmap, double tm_stamp);
  int mkdir_p(string foldername, int mode);
  // for ros_mono_pub.cc
  bool GetLoopDetected();
  bool SetLoopDetected(bool loopDeteced);
  std::vector<KeyFrame*> GetAllKeyFrames();
  bool GetKeyFrameCreated();
  bool SetKeyFrameCreated(bool bTmp);
  cv::Mat GetKeyFramePose();

  // created by zzh over.

 public:
  // Input sensor
  enum eSensor { MONOCULAR = 0, STEREO = 1, RGBD = 2 };
  static bool usedistort_;

 public:
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
  System(const string& strVocFile, const string& strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

  // Proccess the given stereo frame. Images must be synchronized.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // inputRect=true(default) means Images are rectified.
  // Also Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input images: ims[0]: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // ims[1]: depthmap: Float (CV_32F).
  // Returns the camera pose (empty if tracking fails).
  cv::Mat TrackStereo(const vector<cv::Mat>& ims, const double& timestamp, const bool inputRect = true);

  // Proccess the given monocular frame
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  cv::Mat TrackMonocular(const cv::Mat& im, const double& timestamp);

  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();  // used by user, need try

  // Reset the system (clear map)
  void Reset();

  void ShutdownViewer();
  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();  // used by user, like rgbd_tum.cc

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const string& filename, const bool imu_info = false, const bool bgravity_as_w = false);

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const string& filename, const bool bgravity_as_w = false);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const string& filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  std::vector<MapPoint*> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
  const eSensor& GetSensor() const { return mSensor; }

 private:
  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary* mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop detection).
  KeyFrameDatabase* mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Map* mpMap;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints and
  // performs relocalization if tracking fails.
  Tracking* mpTracker;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  LocalMapping* mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
  // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
  LoopClosing* mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  Viewer* mpViewer;

  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;

  // System threads: Local Mapping, Loop Closing(will create a new GBA thread), Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  std::thread* mptLocalMapping;
  std::thread* mptLoopClosing;
  std::thread* mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint*> mTrackedMapPoints;
  // std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;
};

}  // namespace VIEO_SLAM

#endif  // SYSTEM_H
