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

  void SaveMapPCL(const string& filename);

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
  void SaveMap(const string& filename, bool bPCL = false, bool bUseTbc = true, bool bSaveBadKF = false);
  // if read bad KFs, we correct mpTracker->mlpReferences
  bool LoadMap(const string& filename, bool bPCL = false, bool bReadBadKF = false);
  void SaveFrame(string foldername, const cv::Mat& im, const cv::Mat& depthmap, double tm_stamp);
  int mkdir_p(string foldername, int mode);

  // created by zzh over.

 public:
  // Input sensor
  enum eSensor { MONOCULAR = 0, STEREO, RGBD, NUM_SUPPORTED_CAM };
  static bool usedistort_;

 public:
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
  // map_sparse_name != "" for Map Reuse LoadMap(map_sparse_name)
  System(const string& strVocFile, const string& strSettingsFile, const eSensor sensor, const bool bUseViewer = true,
         const string& map_sparse_name = "");

  // 0.usedistort_ = false(default) means total system uses undistorted Images plane; true means distorted/raw image
  // 0.DistCoeffs(config in strSettingsFile) all 0 mean Images are rectified for Stereo.
  // 1.Proccess the given stereo frame. Images must be synchronized.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // 2.Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input images: ims[0]: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale inside.
  // ims[1]: depthmap: Float (CV_32F).
  // 3.Proccess the given monocular frame
  // Input images: ims[0]: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale inside.
  // 0.Returns the camera pose (empty if tracking fails).
  cv::Mat TrackStereo(const vector<cv::Mat>& ims, const double& timestamp);

  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();
  // This SaveMap through window
  void SaveMap();
  // This LoadMap through window, no localization mode change
  void LoadMap();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();  // used by user, need try

  // Reset the system (clear map)
  void Reset(bool bsmart = false);

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
  // You can call this right after TrackStereo
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
  Viewer* mpViewer = nullptr;

  FrameDrawer* mpFrameDrawer = nullptr;
  MapDrawer* mpMapDrawer = nullptr;

  // System threads: Loop Closing(will create a new GBA thread), Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  std::thread* mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset = false;
  bool breset_smart_ = false;

  // Change mode flags
  std::mutex mutex_mode_;
  bool bactivate_localization_mode_ = false;
  bool bdeactivate_localization_mode_ = false;
  bool bsave_map_ = false;
  bool bload_map_ = false;
  // map_name_[0] is map_sparse_name_(save), [1] is map_dense_name_(now PCL), [2] for map_sparse_(load)
  vector<string> map_name_ = {"Map.bin", "Map.pcd", ""};

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint*> mTrackedMapPoints;
  std::mutex mMutexState;
};

}  // namespace VIEO_SLAM

#endif  // SYSTEM_H
