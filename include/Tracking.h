/**
 * This file is part of VIEO_SLAM
 */

#ifndef TRACKING_H
#define TRACKING_H

#include "OdomData.h"
#include <chrono>  //for delay control

// created by zzh over.

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "common/common.h"

#include <mutex>

namespace VIEO_SLAM {
class IMUInitialization;  // zzh, for they includes each other

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class GeometricCamera;

class Tracking {
 public:
  template <class T>
  using aligned_list = IMUPreintegrator::aligned_list<T>;

 private:
  // Created by zzh
  // pure encoder edges
  void TrackWithOnlyOdom(bool bMapUpdated);
  // Odom PreIntegration
  template <class OdomData>
  inline bool iterijFind(const listeig(OdomData) & lodom_data, const double &cur_time,
                         typename listeig(OdomData)::const_iterator &iter, const double &err_odomimg,
                         bool bSearchBack = true);
  /* KeyFrame *plastKF: timestamp_/imu preintegration start KF, can be NULL when type=0
   * KeyFrame *pcurKF: &imu preintegration end/storing KF
   * type: 0 for initialize, 1 for inter-Frame PreInt., 2 for inter-KF PreInt.
   * 2 also culls the data in lists whose tm is (mLastKeyFrame.mTimeStamp,mCurrentKeyFrame.mTimeStamp],
   * culling strategy: tm<mtmSyncOdom is discarded & tm>mCurrentFrame.mTimeStamp is reserved in lists & the left is
   * needed for deltax~ij calculation, for the case Measurement_j-1 uses (M(tj)+M(tj-1))/2,
   * we also reserved last left one in 2 lists(especially for Enc);
   * if pLastF & pCurF exit, we use them instead of mLastFrame & mCurrentFrame
   */
  template <class OdomData>
  bool PreIntegration(const int8_t type, Eigen::aligned_list<OdomData> &lodom_data,
                      typename Eigen::aligned_list<OdomData>::const_iterator &iter_lastodom, FrameBase *plastfb,
                      FrameBase *pcurfb, KeyFrame *plastkf, double *plasttm_kf = nullptr, int8_t verbose = 0);
  void PreIntegration(const int8_t type = 0);
  bool TrackWithIMU(bool bMapUpdated);  // use IMU prediction instead of constant velocity/uniform motion model
  // Predict the NavState of Current Frame by IMU
  // use IMU motion model, like motion update/prediction in ekf, if prediction failed(e.g. no imu data) then false
  bool PredictNavStateByIMU(bool bMapUpdated, bool preint = true);
  // IMUPreintegrator GetIMUPreIntSinceLastKF();
  bool TrackLocalMapWithIMU(
      bool bMapUpdated);  // track local map with IMU motion-only BA, if no imu data it degenerates to TrackLocalMap()
  void RecomputeIMUBiasAndCurrentNavstate();  // recompute bias && mCurrentFrame.mNavState when 19th Frame after reloc.

  // Get mVelocity by Encoder data
  bool GetVelocityByEnc(bool bMapUpdated);

  // Flags for relocalization. Create new KF once bias re-computed & flag for preparation for bias re-compute
  bool mbRelocBiasPrepare;  // true means preparing/not prepared
  // 20 Frames are used to compute bias
  vector<Frame *>
      mv20pFramesReloc;  // vector<Frame,Eigen::aligned_allocator<Frame> > mv20FramesReloc used by JW, notice why we
                         // must use Eigen::aligned_allocator(quaterniond in NavState, or Segementation fault)

  // Consts
  // Error allow between "simultaneous" IMU data(Timu) & Image's mTimeStamp(Timg): Timu=[Timg-err,Timg+err]
  double mdErrIMUImg;
  double tm_shift_ = 0.005;
  // Tbc,Tbo
  cv::Mat mTbc, mTce;  // Tbc is from IMU frame to camera frame;Tbo is from IMU frame to encoder frame(the centre of 2
                       // driving wheels, +x pointing to forward,+z pointing up)
  // delay time of Odom Data received(CacheOdom) relative to the Image entering(GrabImageX), or Camera.delayForPolling
  double mDelayCache;
  // delay time of the image's timestamp to odom's timestamp, or delaytoimu, delaytoenc: Timg=Todom+mDelayToOdom
  double mDelayToIMU, mDelayToEnc;
  // Variables
  std::chrono::steady_clock::time_point
      mtmGrabDelay;  // for delay control(we found our Enc&IMU's response has some delay=20ms)
  // cache queue for vl,vr/IMU & its own timestamp from LastKF
  listeig(EncData) mlOdomEnc;
  listeig(IMUData) mlOdomIMU;
  std::mutex mMutexOdom;                          // for 2 lists' multithreads' operation
  listeig(EncData)::const_iterator miterLastEnc;  // Last EncData pointer in LastFrame, need to check its tm and some
                                                  // data latter to find the min|mtmSyncOdom-tm| s.t. tm<=mtmSyncOdom
  listeig(IMUData)::const_iterator
      miterLastIMU;  // Last IMUData pointer in LastFrame, we don't change the OdomData's content
  bool brecompute_kf2kfpreint_[2];
  bool blast_kf2kfpreint_;

  unsigned long mnLastOdomKFId;

 public:
  // Add Odom(Enc/IMU) data to cache queue
  cv::Mat CacheOdom(const double &timestamp, const double *odomdata, const char mode);

  void SetLastKeyFrame(KeyFrame *pKF) { mpLastKeyFrame = pKF; }
  void SetReferenceKF(KeyFrame *pKF) { mpReferenceKF = pKF; }

  // for ros_mono_pub.cc
  bool mbKeyFrameCreated;
  cv::Mat GetKeyFramePose() { return mpReferenceKF->GetPose(); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // created by zzh over.

 public:
  Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
           KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor);
  ~Tracking() {
    for (auto &pcam : mpCameras) {
      delete pcam;
    }
  }

  // Preprocess the input and call Track(). Extract features and performs stereo matching.
  cv::Mat GrabImageStereo(const vector<cv::Mat> &ims, const double &timestamp, const bool inputRect = true);
  cv::Mat GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);
  cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

  void SetLocalMapper(LocalMapping *pLocalMapper);
  void SetLoopClosing(LoopClosing *pLoopClosing);
  void SetIMUInitiator(IMUInitialization *pIMUInitiator) { mpIMUInitiator = pIMUInitiator; }  // zzh
  void SetViewer(Viewer *pViewer);

  // Load new settings
  // The focal lenght should be similar or scale prediction will fail when projecting points
  // TODO: Modify MapPoint::PredictScale to take into account focal lenght

  // Use this function if you have deactivated local mapping and you only want to localize the camera.
  void InformOnlyTracking(const bool &flag);

 public:
  // Tracking states
  enum eTrackingState {
    SYSTEM_NOT_READY = -1,  // used by FrameDrawer, not Tracking
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3,
    ODOMOK = 4,          // added by zzh, like RECENTLY_LOST in ORB3
    MAP_REUSE = 5,       // added by zzh
    MAP_REUSE_RELOC = 6  // added by zzh
  };

  eTrackingState mState;
  eTrackingState mLastProcessedState;
  double timestamp_lost_, time_recently_lost = 5;

  // Input sensor
  int mSensor;  // value's type is VIEO_SLAM::System::eSensor

  // Current Frame
  EncPreIntegrator preint_enc_kf_;
  IMUPreintegrator preint_imu_kf_;
  double lasttm_preint_kf_[2];
  Frame mCurrentFrame;
#ifdef TIMER_FLOW
  Timer timer_;
#endif
  vector<cv::Mat> mImGrays = vector<cv::Mat>(1);  // used by FrameDrawer

  // Initialization Variables (Monocular)
  std::vector<int> mvIniLastMatches;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatched;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

  // Lists used to recover the full camera trajectory at the end of the execution.
  // Basically we store the reference keyframe for each frame and its relative transformation
  list<cv::Mat> mlRelativeFramePoses;
  list<Vector3d> relative_frame_bvwbs_;
  list<KeyFrame *> mlpReferences;
  list<double> mlFrameTimes;
  list<bool> mlbLost;  // true for lost!

  // True if local mapping is deactivated and we are performing only localization
  bool mbOnlyTracking;

  void Reset();

 protected:
  // Main tracking function. It is independent of the input sensor.
  void Track(cv::Mat img[2] = NULL);  // img[2] recorded by KFs

  // Map initialization for stereo and RGB-D
  void StereoInitialization(cv::Mat img[2] = NULL);

  // Map initialization for monocular
  void MonocularInitialization();
  void CreateInitialMapMonocular();

  void CheckReplacedInLastFrame();
  // track mCurrentFrame with mpReferenceKF by SBB and motion-only BA(if nmatches is enough), then discard outliers,
  // return nInliers>=10
  bool TrackReferenceKeyFrame(int thInMPs = 10, int thMatch = 15);
  // update last Frame's Pose by reference KeyFrame&&mlRelativeFramePoses for nonlocalization mode
  void UpdateLastFrame();
  // UpdateLastFrame, use SBP to get mCurrentFrame.mvpMapPoints, then motion-only BA(if nmatches is enough),discard
  // outliers, return nInliers>=10 for nonlocalization mode
  bool TrackWithMotionModel();

  bool Relocalization();

  void UpdateLocalMap();  // mpMap->SetReferenceMapPoints(mvpLocalMapPoints), UpdateLocalKeyFrames&&UpdateLocalPoints
  // use mvpLocalKeyFrames[i]->mvpMapPoints to fill mvpLocalMapPoints(avoid duplications by
  // pMP->mnTrackReferenceForFrame)
  void UpdateLocalPoints();
  // use mCurrentFrame&&its covisible KFs(>=1 covisible MP)&&the KFs' neighbors(10 best covisibility
  // KFs&&parent&&children) to make mvpLocalKeyFrames, update (mCurrentFrame.)mpReferenceKF to max covisible KF
  void UpdateLocalKeyFrames();

  // use UpdateLocalMap&&SearchLocalPoints(mvpLocalMapPoints) to add new matched mvpMapPoints in mCurrentFrame, then
  // motion-only BA to add Pose's accuracy and update mnMatchesInliers&&pMP->mnFound, finally may judge mnMatchesInliers
  // to one ballot veto the mState to make it Lost
  bool TrackLocalMap();
  // update mCurrentFrame->mvpMapPoints(also discard bad ones)+mvpLocalMapPoints' pMP->mnVisible&&mnLastFrameSeen and
  // call mCurrentFrame.isInFrustum(pMP,0.5), then try to match mvpLocalMapPoints to mCurrentFrame by SBP()(add some
  // mvpMapPoints)
  void SearchLocalPoints();

  bool NeedNewKeyFrame();
  void CreateNewKeyFrame(cv::Mat img[2] = NULL);

  // In case of performing only localization, this flag is true when there are no matches to
  // points in the map. Still tracking will continue if there are enough matches with temporal points.
  // In that case we are doing visual odometry. The system will try to do relocalization to recover
  // "zero-drift" localization to the map.
  bool mbVO;

  // Other Thread Pointers
  LocalMapping *mpLocalMapper;
  LoopClosing *mpLoopClosing;
  IMUInitialization *mpIMUInitiator;  // zzh

  // ORB
  vector<ORBextractor *> mpORBextractors = vector<ORBextractor *>(1, nullptr);
  ORBextractor *mpIniORBextractor;

  // BoW
  ORBVocabulary *mpORBVocabulary;
  KeyFrameDatabase *mpKeyFrameDB;

  // Initalization (only for monocular)
  Initializer *mpInitializer;

  // Local Map
  KeyFrame *mpReferenceKF;  // corresponding to mCurrentFrame(most of time ==mCurrentFrame.mpReferenceKF)
  std::vector<KeyFrame *> mvpLocalKeyFrames;
  std::vector<MapPoint *> mvpLocalMapPoints;

  // System
  System *mpSystem;

  // Drawers
  Viewer *mpViewer;
  FrameDrawer *mpFrameDrawer;
  MapDrawer *mpMapDrawer;

  // Map
  Map *mpMap;

  // Calibration matrix
  cv::Mat mK;
  cv::Mat mDistCoef;
  float mbf;
  vector<GeometricCamera *> mpCameras;

  // New KeyFrame rules (according to fps)
  int mMinFrames;
  int mMaxFrames;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two keyframes.
  float mThDepth;  // 40b, here TUM use 3.2(m)

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
  float mDepthMapFactor;

  // Current matches in frame
  int mnMatchesInliers;  // rectified in TrackLocalMap()
  CREATOR_VAR_MULTITHREADS_INIT(num_track_inliers_, int, , protected, 0)

  // Last Frame, KeyFrame and Relocalisation Info
  KeyFrame *mpLastKeyFrame;
  Frame mLastFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;

  // Motion Model
  cv::Mat mVelocity;

  // Color order (true RGB, false BGR, ignored if grayscale)
  bool mbRGB;

  list<MapPoint *> mlpTemporalPoints;
};

// created by zzh
template <class EncData>
bool Tracking::iterijFind(const listeig(EncData) & mlOdomEnc, const double &cur_time,
                          typename listeig(EncData)::const_iterator &iter, const double &err_odomimg,
                          bool bSearchBack) {
  typename listeig(EncData)::const_iterator iter1;  // iter-1 or iter+1
  double minErr;                                    //=err_odomimg+1;
  if (bSearchBack) {                                // should==mlOdomEnc.end()
    if (iter == mlOdomEnc.end() || iter->mtm > cur_time) {
      while (iter != mlOdomEnc.begin()) {      // begin is min >= lastKFTime-err
        if ((--iter)->mtm <= cur_time) break;  // get max <= cur_time+0
      }
    }
    // Notice mlOdomEnc.empty()==false!
    //     if (iter!=mlOdomEnc.end()){//find a nearest iter to cur_time
    if (cur_time > iter->mtm) {  // iter==begin may still have iter->mtm>cur_time
      minErr = cur_time - iter->mtm;
      iter1 = iter;
      ++iter1;  // iter+1
      if (iter1 != mlOdomEnc.end() && (iter1->mtm - cur_time < minErr)) {
        iter = iter1;
        minErr = iter1->mtm - cur_time;
      }
    } else {  // we don't need to compare anything when iter->mtm>=cur_time
      minErr = iter->mtm - cur_time;
    }
    //     }//==mlOdomEnc.end(), we cannot test ++iter
  } else {                               // should==mlOdomEnc.begin()
    while (iter != mlOdomEnc.end()) {    // begin is min >= lastKFTime-err
      if (iter->mtm >= cur_time) break;  // get min >= lastKFTime+0
      ++iter;
    }
    if (iter != mlOdomEnc.end()) {
      minErr = iter->mtm - cur_time;
      if (iter != mlOdomEnc.begin()) {
        iter1 = iter;
        --iter1;  // iter-1
        if (cur_time - iter1->mtm < minErr) {
          iter = iter1;
          minErr = cur_time - iter1->mtm;
        }
      }
    } else {  //==mlOdomEnc.end(), but we can test --iter
      // Notice mlOdomEnc.empty()==false!
      --iter;
      minErr = cur_time - iter->mtm;
    }
  }
  if (minErr <= err_odomimg) {  // we found nearest allowed iterj/iteri to cur_time/lastKFTime
    return true;
  }
  return false;  // if iteri/j is not in allowed err, returned iter points to nearest one to cur_time or end()
}
template <class OdomData>
bool Tracking::PreIntegration(const int8_t type, Eigen::aligned_list<OdomData> &lodom_data,
                              typename Eigen::aligned_list<OdomData>::const_iterator &iter_lastodom, FrameBase *plastfb,
                              FrameBase *pcurfb, KeyFrame *plastkf, double *plasttm_kf, int8_t verbose) {
  using Eigen::aligned_list;
  using Tldata = OdomData;

  double cur_time = pcurfb->mTimeStamp;
  double derr_imuimg = mdErrIMUImg + tm_shift_;
  bool ret = true;
  switch (type) {  // 0/2 will cull 2 Odom lists,1 will shift the pointer
    case 0:  // for 0th keyframe/frame: erase all the data whose tm<=mCurrentFrame.mTimeStamp but keep the last one,
             // like list.clear()
      if (!lodom_data.empty()) {
        typename aligned_list<Tldata>::const_iterator iter = lodom_data.end();
        // we just find the nearest iteri(for next time) to cur_time, don't need to judge if it's true
        iterijFind<OdomData>(lodom_data, cur_time - tm_shift_, iter, derr_imuimg);
        if (verbose)
          cout << redSTR "ID=" << mCurrentFrame.mnId << "; curDiff:" << iter->mtm - cur_time << whiteSTR << endl;

        // retain the nearest allowed OdomData / iteri used to calculate the Enc PreIntegration
        lodom_data.erase(lodom_data.begin(), iter);
        // nearest iterj to cur_time(next time it may not be the nearest iteri to lastKFTime when
        // mlOdom.back().mtm<cur_time); maybe end() but we handle it in the CacheOdom()
        iter_lastodom = lodom_data.begin();
      }
      break;
    case 1:
    case 3:  // only support Frame* pcurf here
      // PreIntegration between 2 frames, use plastfb & pcurfb to be compatible with
      // RecomputeIMUBiasAndCurrentNavstate()
      if (!lodom_data.empty()) {
        double last_time = plastfb->mTimeStamp;
        bool biteri_research = plasttm_kf && *plasttm_kf != plastfb->mTimeStamp;
        typename aligned_list<Tldata>::const_iterator iter = lodom_data.end(), iterj,
                                                      iteri = type == 1 ? iter_lastodom : lodom_data.begin();
        // iterj&iteri both found then calculate delta~xij(phi,p)
        if (iterijFind<OdomData>(lodom_data, cur_time + tm_shift_, iter, derr_imuimg) &&
            iterijFind<OdomData>(lodom_data, last_time - tm_shift_, iteri, derr_imuimg, false)) {
          //                        assert((miter_lastodom->mtm-last_time)==0&&(iter->mtm-curFTime)==0);
          //                        cout<<redSTR"ID="<<pcurfb->mnId<<"; LastDiff:"<<miter_lastodom->mtm-last_time<<",
          //                        curDiff:"<<iter->mtm-curfTime<<whiteSTR<<endl;
          iterj = iter;
          pcurfb->PreIntegration<OdomData>(plastfb, iteri, ++iterj);  // it is optimized without copy

          // maybe if plastkf == plastfb, we could speed up/skip this preintegration here
          if (plastkf) {
            assert(plasttm_kf);
            Frame *pcurf = dynamic_cast<Frame *>(pcurfb);

            double last_time2 = *plasttm_kf;
            double cur_time2 = cur_time;
            if (iterijFind<OdomData>(lodom_data, cur_time, iter, derr_imuimg - tm_shift_)) {
              cur_time2 = iter->mtm;
            }
            bool breset_intkf = plastkf->mTimeStamp == last_time2;  // even imu tm curl back
            if (plasttm_kf) *plasttm_kf = cur_time2;
            if (biteri_research) {
              iteri = lodom_data.begin();

              if (iterijFind<OdomData>(lodom_data, last_time2 - tm_shift_, iteri, derr_imuimg, false))
                ret = !pcurf->PreIntegrationFromLastKF<OdomData>(plastkf, last_time2, cur_time2, iteri, iterj,
                                                                 breset_intkf);
              else
                ret = false;
            } else
              ret = !pcurf->PreIntegrationFromLastKF<OdomData>(plastkf, last_time2, cur_time2, iteri, iterj,
                                                               breset_intkf);
          }
        } else
          ret = false;

        iterijFind<OdomData>(lodom_data, cur_time - tm_shift_, iter, derr_imuimg);
        // update miter_lastodom pointing to the nearest(now,not next time) one of this frame / begin for next
        // frame
        if (iter != lodom_data.end())
          iter_lastodom = iter;
        else  // if not exist please don't point to the end()! so we have to check the full restriction of
          // miterLastX, it cannot be --begin() for !lodom_data.empty()
          iter_lastodom = --iter;
      }
      break;
    case 2:
      // PreIntegration between 2 KFs & cull 2 odom lists: erase all the data whose
      // tm<=mpReferenceKF(curKF)->mTimeStamp but keep the last one
      KeyFrame *pcurkf = dynamic_cast<KeyFrame *>(pcurfb);
      if (!lodom_data.empty()) {
        KeyFrame *plastkf2 = dynamic_cast<KeyFrame *>(plastfb);
        double last_time = plastkf2->mTimeStamp;
        // iterj, iteri
        typename aligned_list<Tldata>::const_iterator iter = lodom_data.end(), iteri = lodom_data.begin(), iterj;
        // iterj&iteri both found then calculate delta~xij(phi,p)
        if (iterijFind<OdomData>(lodom_data, cur_time + tm_shift_, iter, derr_imuimg) &&
            iterijFind<OdomData>(lodom_data, last_time - tm_shift_, iteri, derr_imuimg, false)) {
          // save odom data list in curKF for KeyFrameCulling()
          iterj = iter;
          iterijFind<OdomData>(lodom_data, cur_time - tm_shift_, iter, derr_imuimg);
          pcurkf->SetPreIntegrationList<Tldata>(iter, ++iterj);
          lodom_data.erase(lodom_data.begin(), iteri);
          pcurkf->AppendFrontPreIntegrationList(lodom_data, iteri, iter);  // will lodom_data.erase(iteri, iter)
        } else {
          iterijFind<OdomData>(lodom_data, cur_time - tm_shift_, iter, derr_imuimg);
          lodom_data.erase(lodom_data.begin(), iter);
          pcurkf->ClearOdomPreInt<OdomData>();
          ret = false;  // or iterj=iter
        }

        iter_lastodom = lodom_data.begin();

        if (!ret) break;

        // mpLastKeyFrame cannot be bad here for mpReferenceKF hasn't been inserted
        // (SetBadFlag only for before KFs)
        if (plastkf) {
          assert(plasttm_kf);
          double last_time2 = *plasttm_kf;  // near cur_time
          pcurkf->PreIntegrationFromLastKF<OdomData>(plastkf, last_time2, iter, iterj, false, verbose);
        } else
          pcurkf->PreIntegration<OdomData>(plastkf2);
      } else
        pcurkf->ClearOdomPreInt<OdomData>();
      break;
  }
  return ret;
}

}  // namespace VIEO_SLAM

#endif  // TRACKING_H
