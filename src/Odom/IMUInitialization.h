#pragma once

#include <mutex>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "OdomPreIntegrator.h"
#include "common/multithreadbase.h"

namespace VIEO_SLAM {

class KeyFrame;
class Map;
class LocalMapping;
class IMUKeyFrameInitFix;
class FrameBase;

using namespace Eigen;
using namespace std;

class IMUKeyFrameInit;

class IMUInitialization : public MultiThreadBase {  // designed for multi threads
 public:
  // for fast imu init
  typedef enum IMUInitMode {
    kIMUInitDefault,  // fix nothing, lba off
    kIMUInitFixScale,
    kIMUInitFixGravity = 0x1 << 1,  // also fix lba's g, but won't use bgba's prior
    kIMUInitLBA = 0x1 << 2,
    kIMUInitLBAFixSG = 0x7,
    kIMUInitRough = 0x8  // will demand FixGravity && Scale, usually coupled with LBA
  } eIMUInitMode;
  typedef float Tcalc_sgba;

 private:
  // imu init related interface params
  // const
  Map *mpMap;
  bool mbMonocular;

  // TODO: bSensorIMU and mGravityVec is in map
  CREATOR_VAR_MULTITHREADS(SensorEnc, bool, b, private, false);
  // for auto reset judgement of this system, automatically check if IMU exists, for it needs initialization with a
  // quite long period of tracking without LOST
  CREATOR_VAR_MULTITHREADS(SensorIMU, bool, b, private, false);
  CREATOR_VAR_MULTITHREADS(VINSInited, bool, b, private, false)  // if IMU initialization is over
  double mdStartTime;                                            // for reset
  cv::Mat mGravityVec;                                           // gravity vector in world frame
  std::mutex mMutexInitIMU;                                      // for mGravityVec, improved by zzh
  // for copying/cache KFs in IMU initialization thread avoiding KeyFrameCulling()
  CREATOR_VAR_MUTEX(CopyInitKFs, bool, b, false)

  // imu init related inner params
  // print related
  string mTmpfilepath;

  // CREATOR_VAR_MULTITHREADS(UpdatingInitPoses,bool,b)//for last propagation in IMU Initialization to stop adding new
  // KFs in Tracking thread, useless for LocalMapping is stopped
  CREATOR_VAR_MULTITHREADS(InitGBA, bool, b, private,
                           false)  // for last GBA(include propagation) required by IMU Initialization,
                                   // LoopClosing always creates new GBA thread when it's true
  CREATOR_VAR_MULTITHREADS(InitGBAOver, bool, b, private, false)  // for Full BA strategy Adjustments
  CREATOR_VAR_MULTITHREADS(InitGBA2, bool, , private, false)
  CREATOR_VAR_MULTITHREADS(InitGBAPriorCoeff, float, , private, 1)

  // like the part of LocalMapping
  CREATOR_VAR_MULTITHREADS(CurrentKeyFrame, KeyFrame *, p, private, nullptr)  // updated by LocalMapping thread

  bool TryInitVIO(void);
  bool TryInitVIO_zzh(void);

  cv::Mat SkewSymmetricMatrix(const cv::Mat &v) {
    return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1), v.at<float>(2), 0, -v.at<float>(0),
            -v.at<float>(1), v.at<float>(0), 0);
  }

 protected:  // thread related special params
  double mdInitTime, mdFinalTime;
  unsigned int mnSleepTime;

  // stop lba thread and wait signal
  bool bstopped_ = false;
  bool bstop_requested_ = false;
  mutex mutex_stop_;

  // for Stop LocalMapping thread
  LocalMapping *mpLocalMapper = nullptr;

  void ResetIfRequested();

 private:
  // parts for fast imu init
  static int deleteKFs_ret(vector<vector<IMUKeyFrameInit *> *> &vKFsInit);
  static int reduceKFs(const vector<char> &reduced_hids, vector<vector<IMUKeyFrameInit *> *> &vKFsInit,
                       vector<vector<IMUKeyFrameInit *> *> &vKFsInit2, vector<int> &Ns, int &num_handlers,
                       vector<char> &id_cams, vector<char> *id_cams_ref);

 public:
  bool mbUsePureVision;  // for pure-vision+IMU Initialization mode!

  IMUInitialization(Map *pMap, const bool bMonocular, const string &strSettingPath);
  ~IMUInitialization() override {
    // we have to stop thread firstly then ~() this class, then ~base class!
    if (pthread_) {
      Setfinish_request(true);
      if (pthread_->joinable()) pthread_->join();
      delete pthread_;
      pthread_ = nullptr;
    }
  }

  void SetLocalMapper(LocalMapping *pLocalMapper) { mpLocalMapper = pLocalMapper; }

  void Run();

  bool SetCopyInitKFs(bool copying) {
    unique_lock<mutex> lock(mutexCopyInitKFs_);
    if (copying && bCopyInitKFs_) return false;
    bCopyInitKFs_ = copying;
    return true;
  }
  cv::Mat GetGravityVec(void);
  void SetGravityVec(const cv::Mat &mat);

  void RequestStop();
  bool isStopped();
  bool stopRequested();
  bool Stop();
  void Release();

  int n_imu_extra_init_ = 0;

  char verbose = mlog::kVerbDeb;  // mlog::kVerbRel; //
};

class IMUKeyFrameInit {  // a simple/base version of KeyFrame just used for IMU Initialization, not designed for multi
                         // threads
 public:                 // I think it belongs FramePoseBase
  const double timestamp_;  // for ComputePreInt
  cv::Mat mTwc, mTcw;       // for TryInitVIO()&OptimizeInitialGyroBias(),see (9) in VIORBSLAM paper
                            // we don't save mTbc for it's constant

 public:
  Vector3d bg_, ba_;  // bgj_bar,baj_bar: if changed, mIMUPreInt needs to be recomputed; unoptimized part of current
                      // defined mNavState
  IMUPreintegrator mOdomPreIntIMU;  // including mlIMUData, for OptimizeInitialGyroBias()
  IMUKeyFrameInit *mpPrevKeyFrame;  // but it's important for mOdomPreIntIMU computation && KeyFrameCulling()

  IMUKeyFrameInit(KeyFrame &kf);
  virtual ~IMUKeyFrameInit(){};

  cv::Mat &GetTcwRef() { return mTcw; }
  const IMUPreintegrator &GetIMUPreInt(void) const { return mOdomPreIntIMU; }

  void ComputePreInt() {  // 0th frame don't use this function, mpPrevKeyFrame shouldn't be bad
    if (mpPrevKeyFrame == NULL) return;
#ifndef TRACK_WITH_IMU
    mOdomPreIntIMU.PreIntegration(mpPrevKeyFrame->timestamp_, timestamp_);
#else
    mOdomPreIntIMU.PreIntegration(mpPrevKeyFrame->timestamp_, timestamp_, mpPrevKeyFrame->bg_, mpPrevKeyFrame->ba_);
#endif
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // for maybe quaterniond in IMUPreintegrator
};
class IMUKeyFrameInitFix : public IMUKeyFrameInit {
 public:
  Vector3d dbg_, dba_;  // dbgba for kFixMode > kFixNone

  IMUKeyFrameInitFix(KeyFrame &kf);
  ~IMUKeyFrameInitFix(){};

  //        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//not needed for it will inherit the base operator new
};

}  // namespace VIEO_SLAM
