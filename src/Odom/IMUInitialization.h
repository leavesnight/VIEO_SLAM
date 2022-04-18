// created by zzh, inspired by JingWang
#ifndef IMUINITIALIZATION_H
#define IMUINITIALIZATION_H

// zzh defined color cout, must after include opencv2
#define redSTR "\033[31m"
#define brightredSTR "\033[31;1m"
#define greenSTR "\e[32m"
#define brightgreenSTR "\e[32;1m"
#define blueSTR "\e[34m"
#define brightblueSTR "\e[34;1m"
#define yellowSTR "\e[33;1m"
#define brownSTR "\e[33m"
#define azureSTR "\e[36;1m"
#define whiteSTR "\e[0m"

// #include <list>
#include <mutex>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include "OdomPreIntegrator.h"
// #include "KeyFrame.h"
// #include "Map.h"
#include "LocalMapping.h"
#include "common/common.h"

#include <unistd.h>

typedef enum kVerboseLevel { kVerbRel, kVerbDeb, kVerbFull };

namespace VIEO_SLAM {

class KeyFrame;
class Map;
class LocalMapping;
class IMUKeyFrameInitFix;
class FrameBase;

using namespace Eigen;
using namespace std;

class IMUKeyFrameInit;

class IMUInitialization {  // designed for multi threads
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
  string mTmpfilepath;
  double mdInitTime, mdFinalTime;
  unsigned int mnSleepTime;
  double mdStartTime;  // for reset
  // cv::Mat mRwiInit;//unused

  CREATOR_VAR_MULTITHREADS(SensorEnc, bool, b);
  CREATOR_VAR_MULTITHREADS(SensorIMU, bool,
                           b);  // for auto reset judgement of this system, automatically check if IMU exists, for it
                                // needs initialization with a quite long period of tracking without LOST
  CREATOR_VAR_MULTITHREADS(VINSInited, bool, b)  // if IMU initialization is over
  cv::Mat mGravityVec;                           // gravity vector in world frame
  std::mutex mMutexInitIMU;                      // for mGravityVec, improved by zzh
  // double mnVINSInitScale; //scale estimation for Mono, not necessary here

  // for copying/cache KFs in IMU initialization thread avoiding KeyFrameCulling()
  CREATOR_VAR_MUTEX(CopyInitKFs, bool, b)

  // CREATOR_VAR_MULTITHREADS(UpdatingInitPoses,bool,b)//for last propagation in IMU Initialization to stop adding new
  // KFs in Tracking thread, useless for LocalMapping is stopped
  CREATOR_VAR_MULTITHREADS(InitGBA, bool, b)      // for last GBA(include propagation) required by IMU Initialization,
                                                  // LoopClosing always creates new GBA thread when it's true
  CREATOR_VAR_MULTITHREADS(InitGBAOver, bool, b)  // for Full BA strategy Adjustments
  CREATOR_VAR_MULTITHREADS_INIT(InitGBA2, bool, , private, false)
  CREATOR_VAR_MULTITHREADS_INIT(InitGBAPriorCoeff, float, , private, 1)

  // like the part of LocalMapping
  CREATOR_VAR_MULTITHREADS(CurrentKeyFrame, KeyFrame *, p)  // updated by LocalMapping thread
  CREATOR_VAR_MUTEX(Finish, bool, b)                        // checked/get by System.cc
  CREATOR_VAR_MUTEX(FinishRequest, bool, b)                 // requested/set by System.cc
  CREATOR_VAR_MULTITHREADS(Reset, bool, b)                  // for reset Initialization variables
  // const
  Map *mpMap;
  bool mbMonocular;
  LocalMapping *mpLocalMapper;  // for Stop LocalMapping thread&&NeedNewKeyFrame() in Tracking thread

  bool TryInitVIO(void);

  // for fast imu init
  typedef Eigen::Matrix<Tcalc_sgba, Eigen::Dynamic, Eigen::Dynamic> MatrixXXcalc;
  typedef Eigen::Matrix<Tcalc_sgba, Eigen::Dynamic, 1> VectorXcalc;
  typedef Eigen::Matrix<Tcalc_sgba, 3, 3> Matrix3calc;
  typedef Eigen::Matrix<Tcalc_sgba, 3, 1> Vector3calc;
  typedef Sophus::SO3ex<Tcalc_sgba> SO3calc;

  typedef enum FixMode {
    kFixNone,
    kFixS = 0x1,       // fix all imus' s
    kFixV = 0x1 << 1,  // fix inited imus' v
    // now inited ones have to has kFixBgBa flag for bg init doesn't implement this flag op.
    kFixBgBa = 0x1 << 2,  // fix inited imus' bgba
    kFixG = 0x1 << 3,     // fix all imus' g
    kFixSVBgBa = kFixS | kFixV | kFixBgBa,
    kFixAll = 0xF
  } eFixMode;
  typedef enum LBAMode {
    kLBAAll,
    kLBAFixPR = 0x1,
    kLBAFixGS = 0x1 << 1,
    kLBAFixBias = 0x1 << 2,
    kLBANone = 0xF
  } eLBAMode;

  int num_imus_ = 1;
  CREATOR_VAR_MULTITHREADS_INIT(InitedIMUs, vector<bool>, b, private, vector<bool>(num_imus_, false))
  CREATOR_VAR_MULTITHREADS_INIT(HasInitedIMU, bool, b, private, false)  // if >=num_imus_ IMU initialization is over
  int TryInitVIO_zzh(void);
  int8_t mode_init_ = kIMUInitLBA;
  static int8_t fopened;
  bool abortBA_ = false;  // now imu init first
  bool tot_imu_inited_ = false;
  // first time enter LBA, which will become Inertial-Only BA
  vector<bool> vfirst_time_ = vector<bool>(num_imus_, true);
  size_t local_window_size_ = 10;

  // parts for fast imu init
  int deleteKFs_ret(vector<vector<IMUKeyFrameInit *> *> &vKFsInit);
  int reduceKFs(const vector<int8_t> &reduced_hids, vector<Vector3d> &bgs_est, vector<Vector3d> &bas_est,
                vector<vector<IMUKeyFrameInit *> *> &vKFsInit, vector<vector<IMUKeyFrameInit *> *> &vKFsInit2,
                vector<int> &Ns, int &num_handlers, vector<int8_t> &id_cams, vector<int8_t> *id_cams_ref);
  // pzeta!=nullptr means step 4 in VIORBSLAM paper IV-C; =nullptr means step 3
  void ConstructAxeqb(const IMUPreintegrator &imupreint12, const IMUPreintegrator &imupreint23,
                      const IMUKeyFrameInit *pKF1, const IMUKeyFrameInit *pKF2, const IMUKeyFrameInit *pKF3,
                      const Matrix3calc &Rcb1, const Matrix3calc &Rcb2, const Vector3calc &pcb1,
                      const Vector3calc &pcb2, const Vector3calc &pcb3, Matrix3calc &phi, Vector3calc &psi,
                      Vector3calc &lambda, bool bcalc_cov_dp = false, Matrix3calc *pzeta = nullptr,
                      const SO3calc *pRwI = nullptr, const Vector3calc *pGI = nullptr, int fixMode = (int)kFixNone);
  // if bcalc_ba=true, add ba term, vice versa; preduced_hids!=nullptr, use fast pid_Tcbs, if preduced_hids=nullptr &&
  // pid_Tcbs, create fast pid_Tcbs
  template <class _FrameBase>
  void FillAxeqb(const vector<_FrameBase *> &fbs_init, int num_var_opt2, int h, MatrixXXcalc *pC, VectorXcalc *pD,
                 int &num_eq2, int fixMode, const double *pscale, bool bcalc_ba, bool bcalc_gdir,
                 int *pnum_eq_h = nullptr, vector<int8_t> *preduced_hids = nullptr,
                 vector<vector<size_t>> *pid_Tcbs = nullptr,
                 Eigen::aligned_vector<Eigen::aligned_vector<SO3calc>> *pRcbs = nullptr,
                 Eigen::aligned_vector<Eigen::aligned_vector<Vector3calc>> *ppcbs = nullptr, int *plast_h = nullptr,
                 const SO3calc *pRwI = nullptr, const Vector3calc *pGI = nullptr, double *psum_dt = nullptr,
                 bool bcalc_cov = true, bool verbose = false);
  // step4: calculate vwbi and scale recover(TODO), lbags means local sliding windows ba optimizes gravity and scale
  void SetIMUInited(vector<bool> &initedIMUs, const vector<int8_t> &id_cams2);
  template <class OdomData>
  int InitIMUv_lbags(const vector<FrameBase *> &vfb_scale_v_b, vector<FrameBase *> &pcurfbs,
                     const vector<unsigned long> &fixed_ref_ids, const vector<int8_t> &id_cams2,
                     vector<bool> &last_imu_inited, const int fixMode, const vector<Vector3d> &bgs_est,
                     const vector<Vector3d> &bas_star, const Eigen::Vector3f &gw, double scale, Map *pMap,
                     unique_lock<mutex> &lock, int8_t mode_lba = (int8_t)kLBAAll, bool bfast_init = false,
                     const vector<typename aligned_list<OdomData>::const_iterator> *pviterbeg = nullptr);
  int InitIMU(vector<vector<IMUKeyFrameInit *> *> &vKFsInit,
              Eigen::aligned_vector<Eigen::aligned_vector<IMUKeyFrameInitFix>> &vKFsFixed, vector<int8_t> &id_cams2,
              vector<bool> &last_imu_inited, const int fixMode, int8_t mode_lba = (int8_t)kLBAAll,
              const vector<Vector6d> &vbgba_init = vector<Vector6d>(),
              const vector<bool> *pdebug_benough_id_cam = nullptr);

  cv::Mat SkewSymmetricMatrix(const cv::Mat &v) {
    return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1), v.at<float>(2), 0, -v.at<float>(0),
            -v.at<float>(1), v.at<float>(0), 0);
  }
  void ResetIfRequested() {
    if (GetReset()) {
      // reset relevant variables
      mdStartTime = -1;
      SetCurrentKeyFrame(NULL);  // SetSensorIMU(false);
      SetVINSInited(false);      // usually this 3 variables are false when LOST then this func. will be called
      SetInitGBA(false);         // if it's true, won't be automatically reset
      SetInitGBAOver(false);
      SetInitGBAPriorCoeff(1);
      SetInitGBA2(false);

      SetInitedIMUs(vector<bool>(num_imus_, false));
      SetHasInitedIMU(false);
      vfirst_time_.clear();  // resize won't change < old_size element!
      vfirst_time_.resize(num_imus_, true);

      tot_imu_inited_ = false;

      if (fopened) fopened = 1;

      SetReset(false);
    }
  }
  CREATOR_SET(Finish, bool, b)
  CREATOR_GET(FinishRequest, bool, b)
 public:
  bool mbUsePureVision;  // for pure-vision+IMU Initialization mode!

  IMUInitialization(Map *pMap, const bool bMonocular, const string &strSettingPath)
      : mpMap(pMap), mbMonocular(bMonocular), mbFinish(true), mbFinishRequest(false), mbReset(false) {
    mbSensorEnc = false;
    mdStartTime = -1;
    mbSensorIMU = false;
    mpCurrentKeyFrame = NULL;
    mbVINSInited = false;
    mbCopyInitKFs = false;
    mbInitGBA = false;
    mbInitGBAOver = false;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    cv::FileNode fnStr = fSettings["test.InitVIOTmpPath"];
    if (!fnStr.empty())
      fnStr >> mTmpfilepath;
    else
      cout << "Nothing recorded for analysis!" << endl;
    // load mbUsePureVision
    cv::FileNode fnSize = fSettings["LocalMapping.LocalWindowSize"];
    if (fnSize.empty()) {
      mbUsePureVision = true;
      cout << redSTR "No LocalWindowSize, then don't enter VIORBSLAM2 or Odom(Enc/IMU) mode!" << whiteSTR << endl;
    } else {
      if ((int)fnSize < 1) {
        mbUsePureVision = true;
        cout << blueSTR "mnLocalWindowSize<1, we use pure-vision+IMU Initialization mode!" << whiteSTR << endl;
      } else
        mbUsePureVision = false;
    }
    cv::FileNode fnTime[3] = {fSettings["IMU.InitTime"], fSettings["IMU.SleepTime"], fSettings["IMU.FinalTime"]};
    if (fnTime[0].empty() || fnTime[1].empty() || fnTime[2].empty()) {
      mdInitTime = 0;
      mnSleepTime = 1e6;
      mdFinalTime = 15;
      cout << redSTR "No IMU.InitTime&SleepTime&FinalTime, we use default 0s & 1s & 15s!" << whiteSTR << endl;
    } else {
      mdInitTime = fnTime[0];
      mnSleepTime = (double)fnTime[1] * 1e6;
      mdFinalTime = fnTime[2];
    }

    if (!mbMonocular) {
      mode_init_ |= kIMUInitFixScale;
    }
  }

  void Run();

  bool SetCopyInitKFs(bool copying) {
    unique_lock<mutex> lock(mMutexCopyInitKFs);
    if (copying && mbCopyInitKFs) return false;
    mbCopyInitKFs = copying;
    return true;
  }

  cv::Mat GetGravityVec(void);
  void SetGravityVec(const cv::Mat &mat);

  CREATOR_GET(Finish, bool, b)
  CREATOR_SET(FinishRequest, bool, b)
  void RequestReset() {  // blocking(3ms refreshing) mode, called by Tracking thread
    SetReset(true);
    for (;;) {
      if (!GetReset()) break;  // if mbReset changes from true to false, resetting is finished
      usleep(3000);
    }
  }
  void SetLocalMapper(LocalMapping *pLocalMapper) { mpLocalMapper = pLocalMapper; }

  char verbose = kVerbDeb;  // kVerbRel; //
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

#endif
