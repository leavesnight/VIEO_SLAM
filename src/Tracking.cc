/**
 * This file is part of VIEO_SLAM
 */

#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Tracking.h"
#include "radtan.h"
#include "KannalaBrandt8.h"
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "common/mlog/log.h"

using namespace std;

namespace VIEO_SLAM {

//#define NO_TRACK_MAP
//#define CHECK_JITTER
//#define NO_LBA_THREAD
//#define DEBUG_STRATEGY
//#define ORB3_STRATEGY_TRACK_BA_ONCE
//#define ORB3_STRATEGY_KF_MORE

cv::Mat Tracking::CacheOdom(const double& timestamp, const double* odomdata,
                            const char mode) {  // different thread from GrabImageX
  // you can add some odometry here for fast Tcw retrieve(e.g. 200Hz)
  unique_lock<mutex> lock(mMutexOdom);
  switch (mode) {
    case System::ENCODER:  // only encoder
      if (mlOdomEnc.empty()) {
        mlOdomEnc.push_back(EncData(odomdata, timestamp + mDelayToEnc));  // notice Timg=Todom+delay
        miterLastEnc = mlOdomEnc.begin();
      } else
        mlOdomEnc.push_back(EncData(odomdata, timestamp + mDelayToEnc));
      mpIMUInitiator->SetSensorEnc(true);
      break;
    case System::IMU:  // only q/awIMU
      if (mlOdomIMU.empty()) {
        mlOdomIMU.push_back(IMUData(odomdata, timestamp + mDelayToIMU));
        miterLastIMU = mlOdomIMU.begin();
      } else
        mlOdomIMU.push_back(IMUData(odomdata, timestamp + mDelayToIMU));
#ifndef TRACK_WITH_IMU
      ;  // mbSensorIMU=false;
#else
      mpIMUInitiator->SetSensorIMU(true);
#endif
      break;
    case System::BOTH:  // both encoder & q/awIMU
      if (mlOdomEnc.empty()) {
        mlOdomEnc.push_back(EncData(odomdata, timestamp + mDelayToEnc));
        miterLastEnc = mlOdomEnc.begin();
      } else
        mlOdomEnc.push_back(EncData(odomdata, timestamp + mDelayToEnc));
      if (mlOdomIMU.empty()) {
        mlOdomIMU.push_back(IMUData(odomdata + 2, timestamp + mDelayToIMU));
        miterLastIMU = mlOdomIMU.begin();
      } else
        mlOdomIMU.push_back(IMUData(odomdata + 2, timestamp + mDelayToIMU));
      mpIMUInitiator->SetSensorEnc(true);
#ifndef TRACK_WITH_IMU
      ;  // mbSensorIMU=false;
#else
      mpIMUInitiator->SetSensorIMU(true);
#endif
      break;
  }

  return cv::Mat();
}

void Tracking::TrackWithOnlyOdom(bool bMapUpdated) {
#ifdef DEBUG_STRATEGY
  CV_Assert(0);
#endif
  if (!mpIMUInitiator->GetVINSInited()) {
    // VEO, we use mVelocity as EncPreIntegrator from LastFrame if EncPreIntegrator exists
    assert(!mVelocity.empty());
    // To avoid accumulated numerical error of pure encoder predictions causing Rcw is not Unit Lie Group (|Rcw|=1)!!!
    cv::Mat Tcw = mVelocity * mLastFrame.GetTcwRef();
    Matrix3d eigRcw = Converter::toMatrix3d(Tcw.rowRange(0, 3).colRange(0, 3));
    cv::Mat Rcw = Converter::toCvMat(Sophus::SO3exd::normalizeRotationM(eigRcw));
    Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
    // if directly use mVelocity*mLastFrame.mTcw), will cause big bug when insertion strategy tends to less insertion!
    mCurrentFrame.SetPose(Tcw);
    // it's difficult to get mCurrentFrame.mvbOutlier like motion-only BA
    mState = ODOMOK;
    cout << greenSTR << mVelocity.at<float>(0, 3) << " " << mVelocity.at<float>(1, 3) << " "
         << mVelocity.at<float>(2, 3) << whiteSTR << endl;
    cout << "ODOM KF: " << mCurrentFrame.ftimestamp_ << endl;
  } else {
    if (mCurrentFrame.GetEncPreInt().mdeltatij > 0) {
      // VIEO, for convenience, we don't use mVelocity as it won't be updated as EncPreIntegrator from LastFrame before
      using namespace Eigen;
      // motion update/prediction by Enc motion model
      const EncPreIntegrator& encpreint = mCurrentFrame.GetEncPreInt();
      // get To1o2:p12 R12
      Vector3d pij(encpreint.mdelxEij.segment<3>(3));
      Matrix3d Rij = Sophus::SO3exd::Exp(encpreint.mdelxEij.segment<3>(0));
      cv::Mat Tij = Converter::toCvSE3(Rij, pij);
      // get Tc2c1
      cv::Mat Tec = Converter::toCvMatInverse(Frame::mTce);
      cv::Mat TlwIE = bMapUpdated ? plast_kf_->GetPose() : mLastFrame.GetTcwRef();
      cv::Mat Tcw = Frame::mTce * Converter::toCvMatInverse(Tij) * Tec *
                    TlwIE;  // To avoid accumulated numerical error of pure encoder predictions
      Matrix3d eigRcw = Converter::toMatrix3d(Tcw.rowRange(0, 3).colRange(0, 3));
      cv::Mat Rcw = Converter::toCvMat(Sophus::SO3exd::normalizeRotationM(eigRcw));
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      mCurrentFrame.SetPose(Tcw);
      mCurrentFrame.UpdateNavStatePVRFromTcw();
      mState = ODOMOK;
      /*
         if (mCurrentFrame.GetIMUPreInt().mdeltatij>0){
           // Pose optimization. false: no need to compute marginalized for current Frame(motion-only), see VIORBSLAM
         paper (4)~(8) if(bMapUpdated){//we call this 2 frames'(FKF/FF) motion-only BA
             //not use Hessian matrix, it's ok
             Optimizer::PoseOptimization(&mCurrentFrame,plast_kf_,mpIMUInitiator->GetGravityVec(),true,true);//fixing
         lastKF(i), optimize curF(j), save its Hessian }else{
             //unfix lastF(j): Hessian matrix exists, use prior Hessian to keep lastF(j)'s Pose stable, optimize j&j+1;
         fix lastF(j): optimize curF(j+1)
             Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame,mpIMUInitiator->GetGravityVec(),true,true);//last F
         unfixed/fixed when lastF.mOdomPreIntIMU.deltatij==0 or RecomputeIMUBiasAndCurrentNavstate(), save its Hessian
           }
           // Discard outliers
           for(int i =0; i<mCurrentFrame.N; i++){
               if(mCurrentFrame.mvpMapPoints[i]&&mCurrentFrame.mvbOutlier[i]){
                 MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                 mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                 mCurrentFrame.mvbOutlier[i]=false;
                 pMP->mbTrackInView = false;
                 pMP->mnLastFrameSeen = mCurrentFrame.mnId;
               }
           }
         }*/
    } else if (mCurrentFrame.GetIMUPreInt().mdeltatij > 0) {
      // motion update/prediction by IMU motion model, but cannot preint twice
      PredictNavStateByIMU(bMapUpdated, false);
      mState = ODOMOK;
    }
    // Update motion model, we don't need to adjust mLastFrame.mTcw.empty() for before ODOMOK, it cannot be LOST(when
    // LOST, PreIntegration(1/3) is not executed/mdeltatij==0, but (2) may be executed if it's a new KF)
    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
    mVelocity = mCurrentFrame.GetTcwRef() * LastTwc;  // Tc2c1/Tcl
    cout << greenSTR << "I/EODOM KF: " << mCurrentFrame.ftimestamp_ << whiteSTR << endl;
  }
}

void Tracking::PreIntegration(const int8_t type) {
  unique_lock<mutex> lock(mMutexOdom);
  PRINT_DEBUG_FILE_MUTEX("type=" << (int)type << "...", mlog::vieo_slam_debug_path, "debug.txt");
  FrameBase *plastfb, *pcurfb;
  if (type == 1 || type == 3) {
    if (type == 1)
      plastfb = static_cast<FrameBase*>(&mLastFrame);
    else
      plastfb = static_cast<FrameBase*>(plast_kf_);
    pcurfb = static_cast<FrameBase*>(&mCurrentFrame);
    bool bpreint;
    if (!blast_kf2kfpreint_ && brecompute_kf2kfpreint_[0])
      bpreint = PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, nullptr);
    else {
      if (blast_kf2kfpreint_) lasttm_preint_kf_[0] = plast_kf_->ftimestamp_;
      bpreint =
          PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, plast_kf_, &lasttm_preint_kf_[0]);
    }
    if (!bpreint)
      brecompute_kf2kfpreint_[0] = true;
    else if (blast_kf2kfpreint_)
      brecompute_kf2kfpreint_[0] = false;
    //   cout<<"!"<<mlOdomIMU.size()<<endl;
    //   cout<<"encdata over"<<endl;
    // TODO(zzh): put it before lock MapUpdate!
    if (!blast_kf2kfpreint_ && brecompute_kf2kfpreint_[1])
      bpreint = PreIntegration<IMUData>(type, mlOdomIMU, miterLastIMU, plastfb, pcurfb, nullptr);
    else {
      if (blast_kf2kfpreint_) lasttm_preint_kf_[1] = plast_kf_->ftimestamp_;
      bpreint =
          PreIntegration<IMUData>(type, mlOdomIMU, miterLastIMU, plastfb, pcurfb, plast_kf_, &lasttm_preint_kf_[1]);
    }
    if (!bpreint)
      brecompute_kf2kfpreint_[1] = true;
    else if (blast_kf2kfpreint_)
      brecompute_kf2kfpreint_[1] = false;
    //   cout<<"over"<<endl;
  } else {
    plastfb = static_cast<FrameBase*>(plast_kf_);
    pcurfb = type == 2 ? static_cast<FrameBase*>(mpReferenceKF) : static_cast<FrameBase*>(&mCurrentFrame);
    KeyFrame* plastkf = brecompute_kf2kfpreint_[0] ? nullptr : plast_kf_;
    PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, plastkf, &lasttm_preint_kf_[0]);
    plastkf = brecompute_kf2kfpreint_[1] ? nullptr : plast_kf_;
    PreIntegration<IMUData>(type, mlOdomIMU, miterLastIMU, plastfb, pcurfb, plastkf, &lasttm_preint_kf_[1]);
    if (plastfb) {
      auto deltatij = pcurfb->GetIMUPreInt().mdeltatij;
      if (deltatij && fabs(pcurfb->ftimestamp_ - plastfb->ftimestamp_ - deltatij) > 1e-5) {
        cout << "dt=" << pcurfb->GetIMUPreInt().mdeltatij << ",hope=" << pcurfb->ftimestamp_ - plastfb->ftimestamp_
             << endl;
        CV_Assert(0 && "preint wrong, check!");
      }
    }
    // won't care initial value of this and how flow strategy it's (like pure vision then visualimu/mixed one)
    for (auto& brecompute : brecompute_kf2kfpreint_) brecompute = true;
    blast_kf2kfpreint_ = true;
  }
  if (type == 2) {
    size_t N = mpReferenceKF->GetListIMUData().size(), N2 = mpReferenceKF->GetListEncData().size();
    PRINT_DEBUG_FILE_MUTEX("List size: " << N << " " << N2 << endl, mlog::vieo_slam_debug_path, "debug.txt");
  }
}
bool Tracking::GetVelocityByEnc(bool bMapUpdated) {
  char type = 1;  // if Map updated, still optimize with last frame
  {
    unique_lock<mutex> lock(mMutexOdom);
    FrameBase *plastfb, *pcurfb;
    plastfb = static_cast<FrameBase*>(&mLastFrame);
    pcurfb = static_cast<FrameBase*>(&mCurrentFrame);
    bool bpreint;
    if (!blast_kf2kfpreint_ && brecompute_kf2kfpreint_[0])
      bpreint = PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, nullptr);
    else {
      if (blast_kf2kfpreint_) lasttm_preint_kf_[0] = plast_kf_->ftimestamp_;
      bpreint =
          PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, plast_kf_, &lasttm_preint_kf_[0]);
    }
    if (!bpreint)
      brecompute_kf2kfpreint_[0] = true;
    else if (blast_kf2kfpreint_)
      brecompute_kf2kfpreint_[0] = false;
  }
  if (mCurrentFrame.GetEncPreInt().mdeltatij == 0) {
    return false;  // check PreIntegration() failed when mdeltatij==0, so mCurrentFrame.mTcw==cv::Mat()
  }

  using namespace Eigen;
  // motion update/prediction by Enc motion model
  const EncPreIntegrator& encpreint = mCurrentFrame.GetEncPreInt();  // problem exits
  // get To1o2:p12 R12
  Vector3d pij(encpreint.mdelxEij.segment<3>(3));
  Matrix3d Rij = Sophus::SO3exd::Exp(encpreint.mdelxEij.segment<3>(0));
  cv::Mat Tij = Converter::toCvSE3(Rij, pij);
  // get Tc2c1
  cv::Mat Tec = Converter::toCvMatInverse(Frame::mTce);
  mVelocity = Frame::mTce * Converter::toCvMatInverse(Tij) * Tec;
  //   Matrix3d eigRcl=Converter::toMatrix3d(mVelocity.rowRange(0,3).colRange(0,3));//we don't need this correction for
  //   this numerical error caused from double to float won't be accumulated! cv::Mat
  //   Rcl=Converter::toCvMat(Sophus::SO3exd::normalizeRotationM(eigRcl));
  //   Rcl.copyTo(mVelocity.rowRange(0,3).colRange(0,3));

  //   cout<<encpreint.mdelxEij[0]<<endl;
  //   cout<<mVelocity.at<float>(0,3)<<" "<<mVelocity.at<float>(1,3)<<" "<<mVelocity.at<float>(2,3)<<endl;
  return true;
}
bool Tracking::TrackWithIMU(bool bMapUpdated) {
  ORBmatcher matcher(0.9, true);  // here 0.9 is useless

  //#ifdef ORB3_STRATEGY_TRACK_BA_ONCE
  // UpdateLastFrame();
  // mLastFrame.UpdateNavStatePVRFromTcw();  // maybe useless
  //#endif

  // Update current frame pose according to last frame or keyframe when last KF is changed by LocalMapping/LoopClosing
  // threads unadded code: Create "visual odometry" points if in Localization Mode
  if (!PredictNavStateByIMU(bMapUpdated)) {
    PRINT_INFO_MUTEX(redSTR "CurF.mdeltatij==0! we have to TrackWithMotionModel()!" << whiteSTR << endl);
    if (!mVelocity.empty()) {
      bool bOK = TrackWithMotionModel();  // maybe track failed then call pure-vision TrackWithMotionModel
      // don't forget to update NavState though bg(bg_bar,dbg)/ba(ba_bar,dba) cannot be updated
      if (bOK) mCurrentFrame.UpdateNavStatePVRFromTcw();
      return bOK;
    } else
      return false;  // all motion model failed
  }
#ifdef ORB3_STRATEGY_TRACK_BA_ONCE
  else
    return true;
#endif

  // fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//already
  // initialized in Frame constructor if this Track function is firstly called

  // Project points seen in previous frame
  int th;
  if (mSensor != System::STEREO)
    th = 15;
  else
    th = 7;
  // has CurrentFrame.mvpMapPoints[bestIdx2]=pMP; in this func. then it can use m-o BA
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR,
                                            mpLocalMapper->th_far_pts_);
  PRINT_DEBUG_FILE("math num2 imu=" << nmatches << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  // If few matches, uses a wider window search
  if (nmatches < 20) {
    auto& curfmps_ref = mCurrentFrame.GetMapPointsRef();
    // it's important for SBP() will not rectify the alreay nice CurretFrame.mvpMapPoints
    fill(curfmps_ref.begin(), curfmps_ref.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR,
                                          mpLocalMapper->th_far_pts_);
  }

  if (nmatches < 10)  // 20)//changed by JingWang
    return false;

#ifdef TIMER_FLOW
  mlog::Timer timer_tmp;
#endif
  // Pose optimization. false: no need to compute marginalized for current Frame(motion-only), see VIORBSLAM paper
  // (4)~(8)
  int num_inliers;
  if (bMapUpdated) {  // we call this 2 frames'(FKF/FF) motion-only BA
    // not use Hessian matrix, it's ok
    // fixing lastKF(i), optimize curF(j)
    num_inliers = Optimizer::PoseOptimization(&mCurrentFrame, plast_kf_, mpIMUInitiator->GetGravityVec(),
#ifndef NO_TRACK_MAP
                                              false);
#else
                                              true);
#endif
  } else {
    //       assert(mLastFrame.mbPrior==true||mLastFrame.mbPrior==false&&(mCurrentFrame.mnId==mnLastRelocFrameId+20||mnLastRelocFrameId==0));
    // unfix lastF(j): Hessian matrix exists, use prior Hessian to keep lastF(j)'s Pose stable, optimize j&j+1; fix
    // lastF(j): optimize curF(j+1)
    // last F unfixed/fixed when lastF.mOdomPreIntIMU.deltatij==0 or RecomputeIMUBiasAndCurrentNavstate(), save its
    // Hessian
    num_inliers = Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, mpIMUInitiator->GetGravityVec(),
#ifndef NO_TRACK_MAP
                                              false);
#else
                                              true);
#endif
  }
  PRINT_DEBUG_FILE("inliers2 imu=" << nmatches << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
#ifdef TIMER_FLOW
  timer_tmp.GetDTfromInit(6, "tracking_thread_debug.txt", "ba1=");
#endif

  // Discard outliers
  int nmatchesMap = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (curfmps[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = curfmps[i];

        mCurrentFrame.EraseMapPointMatch(i);
#ifndef NO_TRACK_MAP
        mCurrentFrame.mvbOutlier[i] = false;
#endif
        auto& trackinfo = pMP->GetTrackInfoRef();
        trackinfo.Reset(&mCurrentFrame);
        nmatches--;
      } else if (curfmps[i]->Observations() > 0)
        nmatchesMap++;
    }
  }
  mnMatchesInliers = nmatchesMap;
  //  cout << "check matchinliers IMU=" << mnMatchesInliers << endl;

  // we haven't designed tracking mode in VIO/VIEO, but we can just consider VIEO map as VEO then
  // use VEO tracking mode! & VIO map as RGBD map then use RGBD tracking mode!
  if (mbOnlyTracking) {
    // change to VO mode if the inlier MapPoint is too few i.e. robot goes to the new environment outside of the given
    // map;old 10
    mbVO = nmatchesMap < 6;
    // Track ok when enough inlier matches;old 20
    return nmatches > 12;
  }

  return nmatchesMap >= 6;  // 10;//Track ok when enough inlier MapPoints, changed by JingWang
}
static void print_debug_navstate(const NavState& ns, const string& prefix) {
  PRINT_DEBUG_FILE(prefix << ns.mpwb.transpose() << ";" << ns.mRwb.log().transpose() << ";" << ns.mvwb.transpose()
                          << ";" << ns.mbg.transpose() << "+" << ns.mdbg.transpose() << ";" << ns.mba.transpose() << "+"
                          << ns.mdba.transpose() << endl,
                   mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
}
bool Tracking::PredictNavStateByIMU(bool bMapUpdated, bool preint) {
  assert(mpIMUInitiator->GetVINSInited());

  // Initialize NavState of mCurrentFrame
  //  Map updated, optimize with last KeyFrame
  NavState& ns = mCurrentFrame.GetNavStateRef();
  print_debug_navstate(mLastFrame.GetNavStateRef(), "lastf's p,r,v,bias=");
  if (bMapUpdated) {
    // Get initial NavState&pose from Last KeyFrame
    ns = plast_kf_->GetNavState();
    // preintegrate from LastKF to curF
    if (preint) PreIntegration(3);

    print_debug_navstate(ns, "lastkf's p,r,v,bias=");
    PRINT_DEBUG_FILE("lastkftm=" << fixed << setprecision(9) << plast_kf_->timestamp_ << endl,
                     mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  }
  // Map not updated, optimize with last Frame
  else {
    // Get initial pose from Last Frame
    ns = mLastFrame.GetNavStateRef();
    // preintegrate from LastF to curF
    if (preint) PreIntegration(1);
    //     cout<<"LastF's pwb="<<ns.mpwb.transpose()<<endl;
  }
  PRINT_DEBUG_FILE("lastftm=" << fixed << setprecision(9) << mLastFrame.timestamp_
                              << ",imupreintdt=" << mCurrentFrame.GetIMUPreInt().mdeltatij << endl,
                   mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  if (mCurrentFrame.GetIMUPreInt().mdeltatij == 0) {
    ns.mbg += ns.mdbg;
    ns.mba += ns.mdba;
    ns.mdbg = ns.mdba =
        Eigen::Vector3d::Zero();  // here we just update bi to bi+dbi for next Frame will use fixedlastF mode
    // notice we can't keep this copy updation of mbi for too long!!!
    return false;  // check PreIntegration() failed when mdeltatij==0, so mCurrentFrame.mTcw==cv::Mat()
  }

  using namespace Eigen;
  // const EncPreIntegrator &encpreint=mCurrentFrame.GetEncPreInt();
  // motion update/prediction by IMU motion model, see VIORBSLAM paper formula(3)
  Eigen::Vector3d gw =
      Converter::toVector3d(mpIMUInitiator->GetGravityVec());        // gravity vector in world Frame/w/C0(not B0!!!)
  const IMUPreintegrator& imupreint = mCurrentFrame.GetIMUPreInt();  // problem exits
  double deltat = imupreint.mdeltatij;

  Eigen::Matrix3d Rwb = ns.getRwb();  // Rwbi
  ns.mpwb += ns.mvwb * deltat + gw * (deltat * deltat / 2) +
             Rwb * (imupreint.mpij + imupreint.mJgpij * ns.mdbg + imupreint.mJapij * ns.mdba);
  ns.mvwb += gw * deltat + Rwb * (imupreint.mvij + imupreint.mJgvij * ns.mdbg + imupreint.mJavij * ns.mdba);
  // don't need to consider numerical error accumulation for it's not continuous multiplication or it will be normalized
  // in BA
  Rwb *=
      imupreint.mRij *
      Sophus::SO3exd::exp(imupreint.mJgRij * ns.mdbg)
          .matrix();  // right update small increment: Rwbj=Rwbi*delta~Rij(bgi)=Rwbi*delta~Rij(bgi_bar)*Exp(JgRij*dbgi)
  ns.setRwb(Rwb);

  // Intialize bj of mCurrentFrame again used as bi_bar of next Frame & update pose matrices from NavState by Tbc
  ns.mbg += ns.mdbg;  // use bj_bar=bi_bar+dbi to set bj_bar(part of motion update, inspired by Random walk)
  ns.mba += ns.mdba;  // notice bi=bi_bar+dbi but we don't update bi_bar again to avoid preintegrating again
  ns.mdbg.setZero();
  ns.mdba.setZero();
  mCurrentFrame.UpdatePoseFromNS();  // for VIE, we may use a complementary filter of encoder & IMU to predict NavState

  print_debug_navstate(ns, "CurF's p,r,v,bias=");

  return true;
}
bool Tracking::TrackLocalMapWithIMU(bool bMapUpdated) {
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();

  SearchLocalPoints();

#ifdef TIMER_FLOW
  mlog::Timer timer_tmp;
#endif
  // Optimize Pose
  if (mCurrentFrame.GetIMUPreInt().mdeltatij == 0) {
    cout << redSTR "CurF.deltatij==0!In TrackLocalMapWithIMU(), Check!" << whiteSTR << endl;
    Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame);  // motion-only BA
    mCurrentFrame.UpdateNavStatePVRFromTcw();  // here is the imu data empty condition after imu's initialized, we must
                                               // update NavState to keep continuous right Tbw after imu's initialized
  } else {
    // mCurrentFrame.UpdatePoseFromNS();
    // 2 frames' motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();
    if (bMapUpdated) {
      // fixed last KF, save its Hessian
      Optimizer::PoseOptimization(&mCurrentFrame, plast_kf_, mpIMUInitiator->GetGravityVec(), true);
    } else {
      //       assert(mLastFrame.mbPrior==true||mLastFrame.mbPrior==false&&(mCurrentFrame.mnId==mnLastRelocFrameId+20||mnLastRelocFrameId==0));
      // last F unfixed/fixed when lastF.mOdomPreIntIMU.deltatij==0 or RecomputeIMUBiasAndCurrentNavstate()
      Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, mpIMUInitiator->GetGravityVec(), true);
      if (mLastFrame.GetIMUPreInt().mdeltatij == 0)
        cout << redSTR "LastF.deltatij==0!In TrackLocalMapWithIMU(), Check!" << whiteSTR << endl;
    }
  }
  // after IMU motion-only BA, we don't change bi to bi+dbi for reason that next Frame may(if imu data exists) still
  // optimize dbi, so it's not necessary to update bi
#ifdef TIMER_FLOW
  timer_tmp.GetDTfromInit(7, "tracking_thread_debug.txt", "ba2=");
#endif

  mnMatchesInliers = 0;
  int nmatches_map_good = 0;

  // Update MapPoints Statistics
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (curfmps[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        curfmps[i]->IncreaseFound();
        if (!mbOnlyTracking) {
          if (curfmps[i]->Observations() > 0) {
            mnMatchesInliers++;
            if (!curfmps[i]->isBad()) ++nmatches_map_good;
          }
        } else {
          mnMatchesInliers++;
          auto obs_tmp = curfmps[i]->Observations();
          if (!curfmps[i]->isBad()) {
            assert(obs_tmp > 0);
            ++nmatches_map_good;
          }
        }
      } else {
        // why not include System::RGBD?maybe or RGBD lba thread can do faster.
#ifdef ORB3_STRATEGY_KF_MORE
        if (mSensor == System::STEREO && !mpIMUInitiator->GetVINSInited()) mCurrentFrame.EraseMapPointMatch(i);
#else
        if (mSensor == System::STEREO) mCurrentFrame.EraseMapPointMatch(i);
#endif
      }
    }
  }

  PRINT_INFO_FILE("inliers_map imu=" << mnMatchesInliers, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  if (nmatches_map_good != mnMatchesInliers)
    PRINT_INFO_FILE(",map_good=" << nmatches_map_good, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  PRINT_INFO_FILE(endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  //  if (mCurrentFrame.ftimestamp_ > 845.064) CV_Assert(0);
  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently (recent 1s)
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
      (mnMatchesInliers < 50))  // 10 || mCurrentFrame.GetIMUPreInt().mdeltatij == 0 && mnMatchesInliers < 50))  // 50)
    return false;

  // ref from ORB3
  if ((mnMatchesInliers > 10) && (mState == ODOMOK)) return true;

  if (mnMatchesInliers < 15)  // 6)  // 30)//notice it's a class data member, changed by JingWang
    return false;
  else {
    double threInliers = 30;  // 15;  // 30;//TODO: check
    if (mCurrentFrame.GetIMUPreInt().mdeltatij == 0 &&
        mnMatchesInliers < threInliers)  // if no imudata then it degenerates to TrackLocalMap()
      return false;
    else
      return true;
  }
}
void Tracking::RecomputeIMUBiasAndCurrentNavstate() {  // see VIORBSLAM paper IV-E
  NavState& nscur = mCurrentFrame.GetNavStateRef();

  // Step1. Estimate gyr bias / see VIORBSLAM paper IV-A
  // compute initial IMU pre-integration for bgi_bar=0 as for the KFs' mOdomPreIntIMU in IMU Initialization
  size_t N = mv20pFramesReloc.size();
  listeig(IMUData)::const_iterator iterTmp = miterLastIMU;
  for (size_t i = 0; i < N - 1; ++i) {
    unique_lock<mutex> lock(mMutexOdom);
    // so vKFInit[i].mOdomPreIntIMU is based on bg_bar=0,ba_bar=0; dbg=0 but dba/ba waits to be optimized
    PreIntegration<IMUData>(1, mlOdomIMU, miterLastIMU, mv20pFramesReloc[i], mv20pFramesReloc[i + 1],
                            nullptr);  // actually we don't need to copy the data list!
  }
  Vector3d bgest;
  Optimizer::OptimizeInitialGyroBias<Frame>(mv20pFramesReloc, bgest);  // though JingWang uses Identity() as Info
  // Update gyr bias of Frames
  assert(N == 20);
  for (size_t i = 0; i < N; ++i) {
    mv20pFramesReloc[i]->GetNavStateRef().mbg = bgest;
    assert(mv20pFramesReloc[i]->GetNavStateRef().mdbg.norm() == 0);
  }
  // Re-compute IMU pre-integration for bgi_bar changes to bgest from 0=>dbgi=0 see VIORBSLAM paper IV
  miterLastIMU = iterTmp;
  for (size_t i = 0; i < N - 1; ++i) {
    unique_lock<mutex> lock(mMutexOdom);
    // so vKFInit[i].mOdomPreIntIMU is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba waits to be optimized
    PreIntegration<IMUData>(1, mlOdomIMU, miterLastIMU, mv20pFramesReloc[i], mv20pFramesReloc[i + 1],
                            nullptr);  // actually we don't need to copy the data list!
  }
  if (!mlOdomEnc.empty()) {  // we update miterLastEnc to current Frame for the next Frame's Preintegration(1/3)!
    unique_lock<mutex> lock(mMutexOdom);
    listeig(EncData)::const_iterator iter = mlOdomEnc.end();
    iterijFind<EncData>(mlOdomEnc, mv20pFramesReloc[N - 2]->ftimestamp_ - tm_shift_, iter, mdErrIMUImg + tm_shift_);
    if (iter != mlOdomEnc.end())
      miterLastEnc = iter;  // update miterLastEnc pointing to the nearest(now,not next time) one of this frame / begin
                            // for next frame
    else
      miterLastEnc = --iter;
    PreIntegration<EncData>(1, mlOdomEnc, miterLastEnc, mv20pFramesReloc[N - 2], mv20pFramesReloc[N - 1], nullptr);
  }

  // Step 3. / See VIORBSLAM paper IV-C&E: Solve C*x=D for x=[ba] (3)x1 vector
  const cv::Mat Tcb = Converter::toCvMatInverse(Frame::mTbc);
  const cv::Mat gw = mpIMUInitiator->GetGravityVec();
  cv::Mat C = cv::Mat::zeros(3 * (N - 2), 3, CV_32F);
  cv::Mat D = cv::Mat::zeros(3 * (N - 2), 1, CV_32F);
  for (int i = 0; i < (int)N - 2; i++) {
    const Frame *pKF2 = mv20pFramesReloc[i + 1], *pKF3 = mv20pFramesReloc[i + 2];
    const IMUPreintegrator &imupreint12 = pKF2->GetIMUPreInt(), &imupreint23 = pKF3->GetIMUPreInt();
    // d means delta
    double dt12 = imupreint12.mdeltatij;
    double dt23 = imupreint23.mdeltatij;
    if (!dt12 || !dt23) continue;
    cv::Mat dp12 = Converter::toCvMat(imupreint12.mpij);
    cv::Mat dp23 = Converter::toCvMat(imupreint23.mpij);
    cv::Mat dv12 = Converter::toCvMat(imupreint12.mvij);
    cv::Mat Jav12 = Converter::toCvMat(imupreint12.mJavij);
    cv::Mat Jap12 = Converter::toCvMat(imupreint12.mJapij);
    cv::Mat Jap23 = Converter::toCvMat(imupreint23.mJapij);
    cv::Mat Twb1 = Converter::toCvMatInverse(mv20pFramesReloc[i]->GetTcwRef()) *
                   Tcb;  // Twbi for pwbi&Rwbi, not necessary for clone()
    cv::Mat Twb2 = Converter::toCvMatInverse(pKF2->GetcvTcwCst()) * Tcb;
    cv::Mat Twb3 = Converter::toCvMatInverse(pKF3->GetcvTcwCst()) * Tcb;
    cv::Mat pb1 = Twb1.rowRange(0, 3).col(3);  // pwbi=pwci_right_scaled+Rwc*tcb
    cv::Mat pb2 = Twb2.rowRange(0, 3).col(3);
    cv::Mat pb3 = Twb3.rowRange(0, 3).col(3);
    cv::Mat Rb1 = Twb1.rowRange(0, 3).colRange(0, 3);  // Rwbi
    cv::Mat Rb2 = Twb2.rowRange(0, 3).colRange(0, 3);
    cv::Mat Rb3 = Twb3.rowRange(0, 3).colRange(0, 3);
    // Stack to C/D matrix; zeta*ba=psi-(lambda*s+phi*dtheta)=psi2, Ci(3*3),Di/psi2(3*1)
    cv::Mat zeta = Rb2 * Jap23 * dt12 + Rb1 * Jav12 * dt12 * dt23 -
                   Rb1 * Jap12 * dt23;  // 3*3 notice here is Jav12, paper writes a wrong Jav23
    // note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
    cv::Mat psi2 = (pb1 - pb2) * dt23 + (pb3 - pb2) * dt12 - Rb2 * dp23 * dt12 -
                   Rb1 * dv12 * dt12 * dt23  // notice here Rwci*pcb+(s=1)*pwci=pwbi
                   + Rb1 * dp12 * dt23 -
                   (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) / 2 * (gw);  // notice here use gw=Rwi*gI-Rwi*gI^
    zeta.copyTo(C.rowRange(3 * i + 0, 3 * i + 3));
    psi2.copyTo(D.rowRange(3 * i + 0, 3 * i + 3));
    // assert(dt12>0&&dt23>0);
  }
  // Use svd to compute C*x=D, x=[ba] 3x1 vector
  cv::Mat w2, u2, vt2;  // Note w2 is 3x1 vector by SVDecomp()
  cv::SVD::compute(C, w2, u2, vt2, cv::SVD::MODIFY_A);
  cv::Mat w2inv = cv::Mat::eye(3, 3, CV_32F);
  for (int i = 0; i < 3; ++i) {
    if (fabs(w2.at<float>(i)) < 1e-10) {
      w2.at<float>(i) += 1e-10;
      cerr << "w2(i) < 1e-10, w=" << endl << w2 << endl;
    }
    w2inv.at<float>(i, i) = 1. / w2.at<float>(i);
  }
  cv::Mat y = vt2.t() * w2inv * u2.t() * D;       // Then y/x = vt'*winv*u'*D
  Vector3d bastareig = Converter::toVector3d(y);  // here bai_bar=0, so dba=ba

  // Update acc bias, not necessary for this program!
  for (size_t i = 0; i < N - 1; i++) {  // for [N-1] is mCurrentFrame
    mv20pFramesReloc[i]->GetNavStateRef().mba = bastareig;
    assert(mv20pFramesReloc[i]->GetNavStateRef().mdba.norm() == 0);
  }

  // Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
  // Compute Velocity of the mCurrentFrame(j/i+1) (but need (i)lastF's Velocity)
  Vector3d pwbjeig, vwbjeig;
  Matrix3d Rwbjeig;
  Frame* pCurF = mv20pFramesReloc[N - 1];
  cv::Mat Twbj = Converter::toCvMatInverse(pCurF->GetcvTcwCst()) * Tcb,
          Twbi = Converter::toCvMatInverse(mv20pFramesReloc[N - 2]->GetcvTcwCst()) * Tcb;
  const IMUPreintegrator& imupreint = pCurF->GetIMUPreInt();  // the same condition as the paper
  double dt = imupreint.mdeltatij;                            // deltatij
  cv::Mat pwbj = Twbj.rowRange(0, 3).col(3);
  pwbjeig = Converter::toVector3d(pwbj);  // pwbj
  Rwbjeig = Converter::toMatrix3d(Twbj.rowRange(0, 3).colRange(0, 3));
  cv::Mat pwbi = Twbi.rowRange(0, 3).col(3), Rwbi = Twbi.rowRange(0, 3).colRange(0, 3);  // pwbi,Rwbi
  cv::Mat vwbi =
      -1. / dt *
      ((pwbi - pwbj) + dt * dt / 2 * gw +
       Rwbi * Converter::toCvMat(
                  Vector3d(imupreint.mpij +
                           imupreint.mJapij * bastareig)));  //-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(deltap~ij+Japij*dbai))
  // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with
  // ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
  vwbjeig = Converter::toVector3d(
      vwbi + gw * dt +
      Rwbi * Converter::toCvMat(
                 Vector3d(imupreint.mvij + imupreint.mJavij * bastareig)));  // vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)

  assert(mv20pFramesReloc[N - 1]->mnId == mCurrentFrame.mnId);

  // Set NavState of Current Frame, P/R/V/bg/ba/dbg/dba
  nscur.mpwb = pwbjeig;
  nscur.setRwb(Rwbjeig);
  nscur.mvwb = vwbjeig;
  nscur.mbg = bgest;
  nscur.mba = bastareig;  // let next PreIntegration() has a fine bi_bar
  nscur.mdbg = nscur.mdba = Vector3d::Zero();
}

// created by zzh over.

Tracking::Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
                   KeyFrameDatabase* pKFDB, const string& strSettingPath, const int sensor)
    : mState(NO_IMAGES_YET),
      mSensor(sensor),
      mbVO(false),
      mpORBVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mpInitializer(static_cast<Initializer*>(NULL)),
      mpSystem(pSys),
      mpViewer(NULL),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mpMap(pMap),
      mnLastRelocFrameId(0),
      mbRelocBiasPrepare(false),
      mnLastOdomKFId(0) {
  // Load camera parameters from settings file
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  // load Tbc,Tbo refer the Jing Wang's configparam.cpp
  cv::FileNode fnT[2] = {fSettings["Camera.Tbc"], fSettings["Camera.Tce"]};
  Eigen::Matrix3d eigRtmp;
  mTbc = cv::Mat::eye(4, 4, CV_32F);
  mTce = cv::Mat::eye(4, 4, CV_32F);
  Frame::mTce = mTce.clone();
  for (int i = 0; i < 2; ++i) {
    if (fnT[i].empty()) {
      PRINT_INFO_MUTEX(redSTR "No Tbc/Tce, please check if u wanna use VIO/VEO/VIEO!" << whiteSTR << endl);
    } else {
      eigRtmp << fnT[i][0], fnT[i][1], fnT[i][2], fnT[i][4], fnT[i][5], fnT[i][6], fnT[i][8], fnT[i][9], fnT[i][10];
      eigRtmp = Eigen::Quaterniond(eigRtmp).normalized().toRotationMatrix();
      if (i == 0) {
        for (int j = 0; j < 3; ++j) {
          mTbc.at<float>(j, 3) = fnT[i][j * 4 + 3];
          for (int k = 0; k < 3; ++k) mTbc.at<float>(j, k) = eigRtmp(j, k);
        }
      } else {
        mTce = cv::Mat::eye(4, 4, CV_32F);
        for (int j = 0; j < 3; ++j) {
          mTce.at<float>(j, 3) = fnT[i][j * 4 + 3];
          for (int k = 0; k < 3; ++k) mTce.at<float>(j, k) = eigRtmp(j, k);
        }
        Frame::mTce = mTce.clone();
      }
    }
  }
  PRINT_INFO_MUTEX("Tbc:" << mTbc << endl);
  Frame::mTbc = mTbc.clone();
  cv::Mat Tcb = Converter::toCvMatInverse(mTbc);
  Frame::meigRcb = Sophus::SO3exd(Converter::toMatrix3d(Tcb.rowRange(0, 3).colRange(0, 3))).matrix();
  Frame::meigtcb = Converter::toVector3d(Tcb.rowRange(0, 3).col(3));
  // load Sigma etad & etawi & gd,ad,bgd,bad & if accelerate needs to *9.81
  cv::FileNode fnSig[3] = {fSettings["IMU.SigmaI"], fSettings["IMU.sigma"], fSettings["IMU.dMultiplyG"]};
  if (fnSig[1].empty() || fnSig[2].empty())
    PRINT_INFO_MUTEX(redSTR "No IMU.sigma or IMU.dMultiplyG!" << whiteSTR << endl);
  else {
    if (fnSig[0].empty()) {
      eigRtmp.setIdentity();
    } else {
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) eigRtmp(i, j) = fnSig[0][i * 3 + j];
    }
    double sigma2tmp[4] = {fnSig[1][0], fnSig[1][1], fnSig[1][2], fnSig[1][3]};
    for (int i = 0; i < 4; ++i) sigma2tmp[i] *= sigma2tmp[i];
    IMUDataDerived::SetParam(eigRtmp, sigma2tmp, fnSig[2],
                             fSettings["IMU.dt_cov_noise_fix"].empty() ? 0 : fSettings["IMU.dt_cov_noise_fix"],
                             fSettings["IMU.freq_hz"].empty() ? 0 : fSettings["IMU.freq_hz"]);
  }
  // load rc,vscale,Sigma etad
  cv::FileNode fnEnc[4] = {fSettings["Encoder.scale"], fSettings["Encoder.rc"], fSettings["Encoder.sigma"]};
  if (fnEnc[0].empty() || fnEnc[1].empty() || fnEnc[2].empty())
    PRINT_INFO_MUTEX(redSTR "No Encoder.simga or Encoder.scale or Encoder.rc!" << whiteSTR << endl);
  else {
    Eigen::Vector2d eig2tmp;
    eig2tmp << fnEnc[2][0], fnEnc[2][1];
    Eigen::DiagonalMatrix<double, 2, 2> Sigma_2tmp(eig2tmp.cwiseProduct(eig2tmp));
    Eigen::Matrix<double, 6, 1> eig6tmp;
    eig6tmp << fnEnc[2][2], fnEnc[2][3], fnEnc[2][4], fnEnc[2][5], fnEnc[2][6], fnEnc[2][7];
    Eigen::DiagonalMatrix<double, 6, 6> Sigma_6tmp(eig6tmp.cwiseProduct(eig6tmp));
    EncData::SetParam(fnEnc[0], fnEnc[1], Sigma_2tmp, Sigma_6tmp,
                      fSettings["Encoder.dt_cov_noise_fix"].empty() ? 0 : fSettings["Encoder.dt_cov_noise_fix"],
                      fSettings["Encoder.freq_hz"].empty() ? 0 : fSettings["Encoder.freq_hz"]);
  }
  // load delay
  cv::FileNode fnDelay[3] = {fSettings["Camera.delaytoimu"], fSettings["Camera.delaytoenc"],
                             fSettings["Camera.delayForPolling"]};
  if (fnDelay[0].empty() || fnDelay[1].empty() || fnDelay[2].empty()) {
    PRINT_INFO_MUTEX(redSTR "No Camera.delaytoimu & delaytoenc & delayForPolling!" << whiteSTR << endl);
    mDelayCache = mDelayToIMU = mDelayToIMU = 0;
  } else {
    mDelayCache = (double)fnDelay[2];
    mDelayToIMU = fnDelay[0];
    mDelayToEnc = fnDelay[1];
  }
  // load mdErrIMUImg
  float fps = fSettings["Camera.fps"];
  if (fps == 0) fps = 30;
  cv::FileNode fnErrIMUImg = fSettings["ErrIMUImg"];
  if (fnErrIMUImg.empty()) {
    PRINT_INFO_MUTEX(redSTR "No ErrIMUImg!" << whiteSTR << endl);
    mdErrIMUImg = 1. / fps;
  } else {
    mdErrIMUImg = (double)fnErrIMUImg;
  }

  // created by zzh over.

  cv::FileNode nodeCameraName = fSettings["Camera.type"];
  string sCameraName = "Pinhole";
  if (!nodeCameraName.empty()) sCameraName = string(nodeCameraName);
  PRINT_INFO_MUTEX("Camera.type=" << sCameraName << endl);
  mpCameras.resize(4);  // TODO: detect the size
  for (int i = 0; i < 4; ++i) {
    bool bexist_cam = false;
    if (sCameraName == "KannalaBrandt8") {
      pSys->usedistort_ = true;  // TODO: now fisheye only usedistort_ is confirmed
      bexist_cam = KannalaBrandt8::ParseCamParamFile(fSettings, i, mpCameras[i]);
    } else {
      if (sCameraName == "Pinhole") {
        assert(!pSys->usedistort_);
        bexist_cam = Pinhole::ParseCamParamFile(fSettings, i, mpCameras[i]);
      } else if (sCameraName == "Radtan") {
        bexist_cam = Radtan::ParseCamParamFile(fSettings, i, mpCameras[i]);
        // pSys->usedistort_ = false;
      } else {
        PRINT_ERR_MUTEX("Unsupported Camera Model in " << __FUNCTION__ << endl);
        exit(-1);
      }
    }
    if (!bexist_cam) {
      mpCameras.resize(i);
      break;
    }
  }
  if (mpCameras.size() > 1) mpFrameDrawer->showallimages_ = true;
  PRINT_INFO_MUTEX("Cam size = " << mpCameras.size() << endl);
  CLEAR_INFO_FILE("start debug:", mlog::vieo_slam_debug_path, "debug.txt");
  CLEAR_INFO_FILE("start tracking debug:", mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  CLEAR_INFO_FILE("start imu init debug:", mlog::vieo_slam_debug_path, "imu_init_thread_debug.txt");
  CLEAR_INFO_FILE("start localmapping debug:", mlog::vieo_slam_debug_path, "localmapping_thread_debug.txt");
  CLEAR_INFO_FILE("start loopclosing thread debug:", mlog::vieo_slam_debug_path, "loopclosing_thread_debug.txt");
  CLEAR_INFO_FILE("start gba thread debug:", mlog::vieo_slam_debug_path, "gba_thread_debug.txt");

  mbf = fSettings["Camera.bf"];

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  PRINT_INFO_MUTEX("- fps: " << fps << endl);

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if (mbRGB)
    PRINT_INFO_MUTEX("- color order: RGB (ignored if grayscale)" << endl);
  else
    PRINT_INFO_MUTEX("- color order: BGR (ignored if grayscale)" << endl);

  // Load ORB parameters

  int nFeatures = fSettings["ORBextractor.nFeatures"];
  float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
  int nLevels = fSettings["ORBextractor.nLevels"];
  int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
  int fMinThFAST = fSettings["ORBextractor.minThFAST"];

  mpORBextractors[0] = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  if (sensor == System::STEREO) {
    int n_cams = mpCameras.size() < 2 ? 2 : mpCameras.size();
    mpORBextractors.resize(n_cams);
    for (int i = 1; i < n_cams; ++i)
      mpORBextractors[i] = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
  }

  if (sensor == System::MONOCULAR)
    mpIniORBextractors[0] = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  PRINT_INFO_MUTEX(endl << "ORB Extractor Parameters: " << endl);
  PRINT_INFO_MUTEX("- Number of Features: " << nFeatures << endl);
  PRINT_INFO_MUTEX("- Scale Levels: " << nLevels << endl);
  PRINT_INFO_MUTEX("- Scale Factor: " << fScaleFactor << endl);
  PRINT_INFO_MUTEX("- Initial Fast Threshold: " << fIniThFAST << endl);
  PRINT_INFO_MUTEX("- Minimum Fast Threshold: " << fMinThFAST << endl);

  auto node_tmp = fSettings["ThDepth"];
  if (!node_tmp.empty()) {
    mThDepth = mbf * (float)node_tmp / mpCameras[0]->getParameters()[0];
  }
  PRINT_INFO_MUTEX(endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl);

  if (sensor == System::RGBD) {
    mDepthMapFactor = fSettings["DepthMapFactor"];
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }
}

void Tracking::SetLocalMapper(LocalMapping* pLocalMapper) { mpLocalMapper = pLocalMapper; }
void Tracking::SetLoopClosing(LoopClosing* pLoopClosing) { mpLoopClosing = pLoopClosing; }
void Tracking::SetViewer(Viewer* pViewer) { mpViewer = pViewer; }

cv::Mat Tracking::GrabImageStereo(const vector<cv::Mat>& ims, const double& timestamp) {
  mtmGrabDelay = chrono::steady_clock::now();  // zzh
  int n_cams = ims.size();
  cv::Mat img_depth_;
  if (System::RGBD == mSensor) {
    // n_cams = 1;
    mpFrameDrawer->showallimages_ = true;
    // no clone here for cvtColor and convertTo will both reallocate memory for Mat
    img_depth_ = ims[1];
    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || img_depth_.type() != CV_32F)
      img_depth_.convertTo(img_depth_, CV_32F, mDepthMapFactor);
  }
  mImGrays.resize(n_cams);
  for (int i = 0; i < n_cams; ++i) {
    mImGrays[i] = ims[i];
    if (mImGrays[i].channels() == 3) {
      if (mbRGB) {
        cvtColor(mImGrays[i], mImGrays[i], CV_RGB2GRAY);
      } else {
        cvtColor(mImGrays[i], mImGrays[i], CV_BGR2GRAY);
      }
    } else if (mImGrays[i].channels() == 4) {
      if (mbRGB) {
        cvtColor(mImGrays[i], mImGrays[i], CV_RGBA2GRAY);
      } else {
        cvtColor(mImGrays[i], mImGrays[i], CV_BGRA2GRAY);
      }
    } else if (mImGrays[i].type() == CV_16UC1) {
      assert(mImGrays[i].channels() == 1 && mImGrays[i].elemSize() == 2);
      mImGrays[i].convertTo(mImGrays[i], CV_8UC1, 10.f * mDepthMapFactor);
    }
  }

#ifdef TIMER_FLOW
  timer_ = mlog::Timer();
#endif
  if (System::STEREO == mSensor)
    mCurrentFrame = Frame(mImGrays, timestamp, mpORBextractors, mpORBVocabulary, mpCameras, mbf, mThDepth,
                          &preint_imu_kf_, &preint_enc_kf_, System::usedistort_, mpLocalMapper->th_far_pts_);
  else if (System::RGBD == mSensor)
    mCurrentFrame = Frame(vector<cv::Mat>({mImGrays[0], img_depth_}), timestamp, mpORBextractors, mpORBVocabulary,
                          mpCameras, mbf, mThDepth, &preint_imu_kf_, &preint_enc_kf_, System::usedistort_);
  else {
    assert(!System::usedistort_ && "Now Mono mode can not support usedistort mode!");
    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
      mCurrentFrame = Frame(mImGrays, timestamp, mpIniORBextractors, mpORBVocabulary, mpCameras, mbf, mThDepth,
                            &preint_imu_kf_, &preint_enc_kf_, false);
    else
      mCurrentFrame = Frame(mImGrays, timestamp, mpORBextractors, mpORBVocabulary, mpCameras, mbf, mThDepth,
                            &preint_imu_kf_, &preint_enc_kf_, false);
  }
#ifdef TIMER_FLOW
  timer_.GetDTfromInit(0, "tracking_thread_debug.txt", "tm curf=");
#endif

  vector<cv::Mat> imgs_dense;
  if (System::RGBD == mSensor)
    for (auto& im : ims) imgs_dense.emplace_back(im.clone());
  Track(imgs_dense);

  return mCurrentFrame.GetTcwRef().clone();
}

// changed a lot by zzh inspired by JingWang
void Tracking::Track(vector<cv::Mat> imgs_dense) {
  if (mState == NO_IMAGES_YET) {
    mState = NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  // Get Map Mutex, we don't hope part of Map to be changed when we write(but some operations like add is
  //  ok) and read it to get new KFs/MPs
  //   core reason is for local map tracking and mono initialize will call GBA with nloopKF=0
  //   and for AddNewKF should have the same map change id with Preintegration lastF/KF's ns.bgba
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
#ifdef TIMER_FLOW
  timer_.GetDTfromInit(3, "tracking_thread_debug.txt", "get lock fromstart=");
#endif

  // delay control
  {
    char sensorType = 0;  // 0 for nothing, 1 for encoder, 2 for IMU, 3 for encoder+IMU
    if (mpIMUInitiator->GetSensorEnc()) ++sensorType;
    if (mpIMUInitiator->GetSensorIMU()) sensorType += 2;
    unique_lock<mutex> lock2(mMutexOdom);
    if (sensorType == 1 && (mlOdomEnc.empty() || mlOdomEnc.back().mtm < mCurrentFrame.ftimestamp_) ||
        sensorType == 2 && (mlOdomIMU.empty() || mlOdomIMU.back().mtm < mCurrentFrame.ftimestamp_) ||
        sensorType == 3 && (mlOdomEnc.empty() || mlOdomEnc.back().mtm < mCurrentFrame.ftimestamp_ ||
                            mlOdomIMU.empty() || mlOdomIMU.back().mtm < mCurrentFrame.ftimestamp_)) {
      lock2.unlock();  // then delay some ms to ensure some Odom data to come if it runs well
      mtmGrabDelay +=
          chrono::duration_cast<chrono::nanoseconds>(chrono::duration<double>(mDelayCache));  // delay default=20ms
      while (chrono::steady_clock::now() < mtmGrabDelay) usleep(1000);                        // allow 1ms delay error
    }  // if still no Odom data comes, deltax~ij will be set unknown
  }

  // Different operation, according to whether the map is updated
  bool bMapUpdated = false;
  static int premapid = 0;
  int curmapid = mpMap->GetLastChangeIdx();
  if (curmapid != premapid) {
    premapid = curmapid;
    bMapUpdated = true;
  }

  PRINT_INFO_FILE("curf tm=" << fixed << setprecision(9) << mCurrentFrame.ftimestamp_ << endl,
                  mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  if (mState == NOT_INITIALIZED) {
    if (mSensor == System::STEREO || mSensor == System::RGBD)
      StereoInitialization(imgs_dense);
    else
      MonocularInitialization();

    mpFrameDrawer->Update(this);

    if (mState != OK) return;
  } else {
    // System is initialized. Track Frame.
    bool bOK;

    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (!mbOnlyTracking) {
      // Local Mapping is activated. This is the normal behaviour, unless
      // you explicitly activate the "only tracking" mode.

      if (mState == OK || mState == ODOMOK) {
        // Local Mapping might have changed some MapPoints tracked in last frame
        CheckReplacedInLastFrame();  // so use the replaced ones

        // if IMU info(bgi,gw,bai) is intialized use IMU motion model
        if (mpIMUInitiator->GetVINSInited()) {
          // but still need >=20 Frames after reloc, bi becomes ok for IMU motion
          // update(calculated in 19th Frame after reloc. KF)
          if (!mbRelocBiasPrepare) {
            bOK = TrackWithIMU(bMapUpdated);
            if (!bOK) {
              cout << redSTR "TrackWitIMU() failed!" << whiteSTR << endl;
              // don't forget to update NavState though bg(bg_bar,dbg)/ba(ba_bar,dba) cannot be updated
              if (bOK = TrackReferenceKeyFrame()) mCurrentFrame.UpdateNavStatePVRFromTcw();
            }
          } else {
            // <20 Frames after reloc then use pure-vision tracking, but notice we can still use Encoder motion update
            // and short time IMU update
            // we don't preintegrate Enc or IMU here for a simple process with mdeltatij==0! when bOK==false
            if (mVelocity.empty()) {
              // match with rKF, use lF as initial & m-o BA
              if (bOK = TrackReferenceKeyFrame()) mCurrentFrame.UpdateNavStatePVRFromTcw();
              cout << fixed << setprecision(6) << redSTR "TrackRefKF()" whiteSTR << " " << mCurrentFrame.ftimestamp_
                   << " " << mCurrentFrame.mnId << " " << (int)bOK << endl;
            } else {
              bOK = TrackWithMotionModel();  // match with lF, use v*lF(velocityMM) as initial & m-o BA
              if (!bOK) {
                bOK = TrackReferenceKeyFrame();
                cout << redSTR "TrackRefKF()2" whiteSTR << " " << mCurrentFrame.ftimestamp_ << " " << mCurrentFrame.mnId
                     << " " << (int)bOK << endl;
              }
              if (bOK) mCurrentFrame.UpdateNavStatePVRFromTcw();
            }
          }
        } else {
          if (!mLastFrame.GetTcwRef().empty())
            GetVelocityByEnc(bMapUpdated);  // try to utilize the Encoder's data
          else
            cout << redSTR << "LastFrame has no Tcw!" << whiteSTR << endl;
#ifdef DEBUG_STRATEGY
          if (mVelocity.empty()) mVelocity = cv::Mat::eye(4, 4, CV_32F);
          cout << "check vel=" << mVelocity.col(3).rowRange(0, 3).t() << ","
               << Sophus::SO3exd(Converter::toMatrix3d(mVelocity.rowRange(0, 3).colRange(0, 3))).log().transpose()
               << endl;
          auto dt = mCurrentFrame.ftimestamp_ - mLastFrame.ftimestamp_;
          CV_Assert(dt < 0.045);
#endif
          // if last frame relocalized, there's no motion could be calculated, so I think 2nd condition is useless
          if (mVelocity.empty()) {  // || mCurrentFrame.mnId<mnLastRelocFrameId+2){
            // if (!mVelocity.empty()) cerr<<redSTR"Error in Velocity.empty()!!!"<<endl;
            bOK = TrackReferenceKeyFrame();  // match with rKF, use lF as initial & m-o BA
            cout << fixed << setprecision(6) << redSTR "TrackRefKF()" whiteSTR << " " << mCurrentFrame.ftimestamp_
                 << " " << mCurrentFrame.mnId << " " << (int)bOK << endl;
          } else {
            bOK = TrackWithMotionModel();  // match with lF, use v*lF(velocityMM) as initial & m-o BA
            if (!bOK) {
              bOK = TrackReferenceKeyFrame();
              cout << redSTR "TrackRefKF()2" whiteSTR << " " << mCurrentFrame.ftimestamp_ << " " << mCurrentFrame.mnId
                   << " " << (int)bOK << endl;
            }
            // cout<<mCurrentFrame.mnId<<endl;
          }
        }
      } else  // mState==LOST or MAP_REUSE
      {
        if (mState == MAP_REUSE) PreIntegration();  // clear cached Odom List
        bOK = Relocalization();
        cout << greenSTR "Relocalization()" whiteSTR << " " << mCurrentFrame.ftimestamp_ << " " << mCurrentFrame.mnId
             << " " << (int)bOK << endl;
      }
    } else {
      // Localization Mode: Local Mapping is deactivated
      if (mState == MAP_REUSE) {
        PreIntegration();
        bOK = Relocalization();
        cout << greenSTR "OnlyTracking Relocalization()" whiteSTR << " " << mCurrentFrame.ftimestamp_ << " "
             << mCurrentFrame.mnId << " " << (int)bOK << endl;
      } else if (mState == LOST) {
        bOK = Relocalization();
        cout << "Lost--Local. Mode" << endl;
      } else if (mpIMUInitiator->GetVINSInited()) {  // if IMU info(bgi,gw,bai) is intialized use IMU motion model
        // Todo: Add VIO & VIEO Tracking Mode, it's not key for our target, so we haven't finished this part
        cout << redSTR << "Entering Wrong Tracking Mode With VIO/VIEO, Please Check!" << endl;
        assert(0);
      } else {
        if (!mLastFrame.GetTcwRef().empty())
          GetVelocityByEnc(bMapUpdated);  // try to utilize the Encoder's data
        else
          cout << redSTR << "LastFrame has no Tcw!" << whiteSTR << endl;
        if (!mbVO) {
          // In last frame we tracked enough MapPoints in the map
          if (!mVelocity.empty()) {
            bOK = TrackWithMotionModel();
          } else {
            bOK = TrackReferenceKeyFrame();
          }
          if (!bOK && mState == ODOMOK) mbVO = true;
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing relocalization.
          // If relocalization is sucessfull we choose that solution, otherwise we retain
          // the "visual odometry" solution.
          bool bOKMM = false;
          bool bOKReloc = false;
          vector<MapPoint*> vpMPsMM;
          vector<bool> vbOutMM;
          cv::Mat TcwMM;
          if (!mVelocity.empty()) {
            bOKMM = TrackWithMotionModel();
            vpMPsMM = mCurrentFrame.GetMapPointMatches();
            vbOutMM = mCurrentFrame.mvbOutlier;
            TcwMM = mCurrentFrame.GetTcwRef().clone();
          }
          bOKReloc = Relocalization();

          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            mCurrentFrame.GetMapPointsRef() = vpMPsMM;
            mCurrentFrame.mvbOutlier = vbOutMM;

            if (mbVO) {
              const auto& curfmps = mCurrentFrame.GetMapPointMatches();
              for (int i = 0; i < mCurrentFrame.N; i++) {
                if (curfmps[i] && !mCurrentFrame.mvbOutlier[i]) {
                  curfmps[i]->IncreaseFound();  //++mnFound in this map point
                }
              }
            }
          } else if (bOKReloc) {
            mbVO = false;
            cout << "Reloc--Local. Mode" << endl;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }
    blast_kf2kfpreint_ = false;  // ensure every time frame process after last kf created can set it false!

    // firstly use last mState==OK Frame.mpReferenceKF, maybe use most covisible KF as the (mCurrentFrame.)mpReferenceKF
    // in TrackLocalMap()
    mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef TIMER_FLOW
    timer_.GetDTfromInit(4, "tracking_thread_debug.txt", "trackmm fromstart=");
#endif
    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (!mbOnlyTracking) {
      if (bOK) {
        // if imu not intialized(including relocalized bias recomputation)
        bool bimu_inited = mpIMUInitiator->GetVINSInited();
        if (!bimu_inited || mbRelocBiasPrepare)
#ifndef NO_TRACK_MAP
          bOK = TrackLocalMap();
        else
          bOK = TrackLocalMapWithIMU(bMapUpdated);
#else
          ;
#endif

        if (!bOK) {
          PRINT_INFO_MUTEX(redSTR "TrackLocalMap() failed! imu_inited=" << (int)bimu_inited << ",reloc_prepare="
                                                                        << (int)mbRelocBiasPrepare << whiteSTR << endl);
        }
#ifdef DEBUG_STRATEGY
        else {
          static set<string> opened_paths;
          string path = "/data_1/home/leavesnight/dataset/vr/5/realtime_trajBE_ham.txt";
          ofstream fout(path, (opened_paths.count(path) ? ios::app : ios::out));
          opened_paths.insert(path);
          fout << fixed << setprecision(9);
          Eigen::Vector3d twi;

          auto ns = mCurrentFrame.GetNavState();
          twi = ns.mpwb;
          Vector4d qwi = ns.mRwb.unit_quaternion().coeffs();
          auto Twc = mCurrentFrame.GetTwc();
          twi = Twc.translation();
          qwi = Twc.so3().unit_quaternion().coeffs();
          fout << mCurrentFrame.ftimestamp_ << " " << twi[0] << " " << twi[1] << " " << twi[2] << " " << qwi.x() << " "
               << qwi.y() << " " << qwi.z() << " " << qwi.w() << endl;
        }
#endif
      }
    } else {
      // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
      // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
      // the camera we will use the local map again.
      if (bOK && !mbVO) bOK = TrackLocalMap();
    }
    mpLocalMapper->Setnum_track_inliers(mnMatchesInliers);

    if (bOK) {
      mState = OK;
      // Add Frames to re-compute IMU bias after reloc
      if (mbRelocBiasPrepare) {
        cout << redSTR << " Relocalization Recomputation Preparing..." << whiteSTR << endl;
        mv20pFramesReloc.push_back(new Frame(mCurrentFrame, true));

        // Before creating new keyframe
        // Use 20 consecutive frames to re-compute IMU bias, see VIORBSLAM paper IV-E
        if (mCurrentFrame.mnId == mnLastRelocFrameId + 20 - 1) {
          RecomputeIMUBiasAndCurrentNavstate();  // Update NavState of CurrentFrame for it's only used in Tracking
                                                 // thread
          // Clear flag and Frame buffer
          mbRelocBiasPrepare = false;
          for (int i = 0; i < mv20pFramesReloc.size(); ++i) delete mv20pFramesReloc[i];
          mv20pFramesReloc.clear();
        }
      }
    } else {
      // if MAP_REUSE, we keep mState until it's relocalized
      if (mState == MAP_REUSE) return;

      if (OK == mState) timestamp_lost_ = mCurrentFrame.ftimestamp_;

      // use Odom data to get mCurrentFrame.mTcw
      // though it may introduce error, it ensure the completeness of the Map
      if (OK == mState || ODOMOK == mState) {
        if (mCurrentFrame.GetEncPreInt().mdeltatij > 0 ||
            (mCurrentFrame.GetIMUPreInt().mdeltatij > 0 &&
             mCurrentFrame.ftimestamp_ - timestamp_lost_ <= time_recently_lost)) {
          TrackWithOnlyOdom(bMapUpdated);
        } else {
          // for when enc ok, always OK/ODOMOK, no mbRelocBiasPrepare entered for no relocalization is called or
          // always Preintegration(1/3) before, but when enc failed we need to preint kf2kf next relocalization time
          for (auto& brecompute : brecompute_kf2kfpreint_) brecompute = true;

          // Clear Frame vectors for reloc bias computation
          if (mv20pFramesReloc.size() > 0) {
            for (int i = 0; i < mv20pFramesReloc.size(); ++i) delete mv20pFramesReloc[i];
            mv20pFramesReloc.clear();
          }

          // if LOST, the system can only be recovered through relocalization module, so no need to set
          // mbRelocBiasPrepare
          mState = LOST;
        }
      }
    }
#ifdef TIMER_FLOW
    timer_.GetDTfromInit(5, "tracking_thread_debug.txt", "tracklocalmp fromstart=");
#endif

    // Update drawer
#ifndef MUTE_VIEWER
    mpFrameDrawer->Update(this);
#endif

    // If tracking were good, check if we insert a keyframe
    if (bOK) {
      // Update motion model
      if (!mLastFrame.GetTcwRef().empty()) {
        cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
        mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
        mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
        mVelocity = mCurrentFrame.GetTcwRef() * LastTwc;  // Tc2c1/Tcl
      } else {
        mVelocity = cv::Mat();  // can use odometry data here!
        // cout<<redSTR"Error in mVelocity=cv::Mat()"<<whiteSTR<<endl;
      }

#ifndef MUTE_VIEWER
      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetTcwRef());
#endif

      // Clean VO matches, related to Localization mode
      const auto& curfmps = mCurrentFrame.GetMapPointMatches();
      for (int i = 0; i < mCurrentFrame.N; i++) {
        MapPoint* pMP = curfmps[i];
        if (pMP)                        // if the mappoint of th i feature is added in to the pMap
          if (pMP->Observations() < 1)  // if now no KF observes it, delete it from the vmap in Frame
          {
            // though motion-only BA will initialize this, here is for new MapPoints created by CreateNewKeyFrame(),
            // they must be inliers
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.EraseMapPointMatch(i);
          }
      }

      // Delete temporal MapPoints, for they're only used for current Tcw calculation, related to Localization mode
      for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend;
           lit++) {
        MapPoint* pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

      // Check if we need to insert a new keyframe!!
      if (NeedNewKeyFrame()) {
        // only create the only CurrentFrame viewed MapPoints without inliers+outliers in mpMap, to avoid possibly
        // replicated MapPoints
        CreateNewKeyFrame(imgs_dense);
      }

      // We allow points with high innovation (considererd outliers by the Huber Function)
      // pass to the new keyframe, so that bundle adjustment will finally decide
      // if they are outliers or not. We don't want next frame to estimate its position
      // with those points so we discard them in the frame.
      const auto& curfmps2 = mCurrentFrame.GetMapPointMatches();
      for (int i = 0; i < mCurrentFrame.N; i++) {        // new created MPs' mvbOutlier[j] is default false
        if (curfmps2[i] && mCurrentFrame.mvbOutlier[i])  // delete the final still outliers mappoints in Frame
          mCurrentFrame.EraseMapPointMatch(i);
      }

      // For the 1st frame's tracking strategy after IMU initialized(notice no prior IMU Hessian matrix),
      //  JingWang chooses tracking with lastKF, If the transition frame's tracking is unstable, we can improve it
      //  through our no Hessian matrix strategy~
    } else if (mState == ODOMOK) {  // if it's lost in Camera mode we use Odom mode
      // not necessary to update motion model for mVelocity is already got through odom data

#ifndef MUTE_VIEWER
      mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetTcwRef());
#endif

      // Clean VO matches, related to Localization mode
      const auto& curfmps = mCurrentFrame.GetMapPointMatches();
      for (int i = 0; i < mCurrentFrame.N; i++) {
        MapPoint* pMP = curfmps[i];
        if (pMP)                        // if the mappoint of th i feature is added in to the pMap
          if (pMP->Observations() < 1)  // if now no KF observes it, delete it from the vmap in Frame
          {
            // though motion-only BA will initialize this, here is for new MapPoints created by CreateNewKeyFrame(),
            // they must be inliers
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.EraseMapPointMatch(i);
          }
      }

      if (NeedNewKeyFrame()) {
        // only create the only CurrentFrame viewed MapPoints without inliers+outliers in mpMap, to avoid possibly
        // replicated MapPoints
        CreateNewKeyFrame(imgs_dense);
      }
      const auto& curfmps2 = mCurrentFrame.GetMapPointMatches();
      for (int i = 0; i < mCurrentFrame.N; i++) {        // new created MPs' mvbOutlier[j] is default false
        if (curfmps2[i] && mCurrentFrame.mvbOutlier[i])  // delete the final still outliers mappoints in Frame
          mCurrentFrame.EraseMapPointMatch(i);
      }
    }

    // Reset if the camera get lost soon after initialization
    if (!mbOnlyTracking && mState == LOST) {
      bool autoreset = mpIMUInitiator->GetSensorIMU() && !mpIMUInitiator->mbUsePureVision
                           ? !mpIMUInitiator->GetVINSInited()
                           : mpMap->KeyFramesInMap() <= 5;
      if (autoreset) {
        PRINT_INFO_MUTEX(redSTR "Track lost soon after initialisation, reseting..." << whiteSTR << endl);
        mpSystem->Reset();
        return;
      }
    }

    // when mCurrentFrame is not KF but it cannot see common MPs in local MPs(e.g. it has all new MPs but it's not
    // inserted as new KF & you can get its mTcw through other odometry)
    if (!mCurrentFrame.mpReferenceKF) mCurrentFrame.mpReferenceKF = mpReferenceKF;

    // copy constructor for some Mat.clone() and vec<>[][] deep copy, notice mLastFrame is also set in
    // XXXInitialization()!
    mLastFrame = Frame(mCurrentFrame, true);
  }

  // Store frame pose information to retrieve the complete camera trajectory afterwards.
  if (!mCurrentFrame.GetTcwRef().empty()) {
    // when it's lost but get an initial pose through motion-only BA , it can still recover one low-quality estimation,
    // used in UpdateLastFrame()
    cv::Mat Tcr = mCurrentFrame.GetTcwRef() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
    mlRelativeFramePoses.push_back(Tcr);
    const NavStated& ns = mCurrentFrame.GetNavStateRef();
    relative_frame_bvwbs_.push_back(ns.mRwb.inverse() * ns.mvwb);
    mlpReferences.push_back(mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame.ftimestamp_);
    // false if it isn't lost, when it has Tcw, it stll can be LOST for not enough inlier MapPoints
    mlbLost.push_back(mState == LOST);
  } else {
    // This can happen if tracking is lost
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());  // I think actually it's unused
    relative_frame_bvwbs_.push_back(relative_frame_bvwbs_.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState == LOST);  // it's key value to judge
  }
}

void Tracking::StereoInitialization(vector<cv::Mat> imgs_dense) {
  PreIntegration();  // PreIntegration Intialize, zzh

  if (mCurrentFrame.N > 500) {
    // Set Frame pose to the origin
    mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
    PRINT_INFO_MUTEX("init tm=" << mCurrentFrame.ftimestamp_ << endl);

    // Create KeyFrame
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, true);
    pKFini->imgs_dense_ = imgs_dense;

    // Insert KeyFrame in the map
    mpMap->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    // every points with depth info!
    if (mCurrentFrame.stereoinfo_.v3dpoints_.empty()) {
      for (int i = 0; i < mCurrentFrame.N; i++) {
        float z = mCurrentFrame.stereoinfo_.vdepth_[i];
        if (z > 0) {
          cv::Mat x3D =
              mCurrentFrame.UnprojectStereo(i);  // use camera intrinsics to backproject the piont from (u,v) to x3Dw
          MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
          pNewMP->AddObservation(pKFini,
                                 i);  // add which KF with the order of features in it has observed this mappoint
          pKFini->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();  // choose the observed descriptor having the least median distance
                                                    // to the left
          pNewMP->UpdateNormalAndDepth();           // calc the mean viewing direction and max/min dist?
          mpMap->AddMapPoint(pNewMP);

          mCurrentFrame.AddMapPoint(pNewMP, i);  // the realization of the AddMapPiont in class Frame
        }
      }
    } else {
      // TODO: only left this
      for (int k = 0; k < mCurrentFrame.stereoinfo_.v3dpoints_.size(); ++k) {
        if (mCurrentFrame.stereoinfo_.goodmatches_[k]) {
          int i = mCurrentFrame.mapidxs2n_[k];
          CV_Assert(-1 != i);
          cv::Mat x3D =
              mCurrentFrame.UnprojectStereo(i);  // use camera intrinsics to backproject the piont from (u,v) to x3Dw
          MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
          auto idxs = mCurrentFrame.mvidxsMatches[k];
          bool icheck = false;
          PRINT_DEBUG_FILE_MUTEX("addmap-1, kf0:", mlog::vieo_slam_debug_path, "debug.txt");
          CV_Assert(idxs.size());
          for (int cami = 0; cami < idxs.size(); ++cami) {
            if (-1 != idxs[cami]) {
              auto icami = mCurrentFrame.mapin2n_[cami][idxs[cami]];
              if (icami == i) icheck = true;
              // add which KF with the order of features in it has observed this mappoint
              pNewMP->AddObservation(pKFini, icami);
              pKFini->AddMapPoint(pNewMP, icami);
              mCurrentFrame.AddMapPoint(pNewMP, icami);  // the realization of the AddMapPiont in class Frame
            }
          }
          PRINT_DEBUG_FILE_MUTEX(endl, mlog::vieo_slam_debug_path, "debug.txt");
          assert(icheck);
          pNewMP->ComputeDistinctiveDescriptors();  // choose the observed descriptor having the least median distance
                                                    // to the left
          pNewMP->UpdateNormalAndDepth();           // calc the mean viewing direction and max/min dist?
          mpMap->AddMapPoint(pNewMP);
        }
      }
    }

    PRINT_INFO_MUTEX("New map created with " << mpMap->MapPointsInMap() << " points" << endl);

    mpLocalMapper->InsertKeyFrame(pKFini);  // add it to the local mapper KF list

    mCurrentFrame.mpReferenceKF = pKFini;  // I think it should be put before mLastFrame=!
    mLastFrame = Frame(mCurrentFrame, true);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    plast_kf_ = pKFini;

    mvpLocalMapPoints = mpMap->GetAllMapPoints();  // for MapDrawer.cc
    mpReferenceKF = pKFini;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

#ifndef MUTE_VIEWER
    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetTcwRef());
#endif

    mState = OK;
  }
}

void Tracking::MonocularInitialization() {
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {
      PreIntegration();  // PreIntegration Intialize, Clear IMUData buffer, zzh

      mInitialFrame = Frame(mCurrentFrame, true);
      mLastFrame = Frame(mCurrentFrame, true);
      assert(!mCurrentFrame.usedistort_);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++) mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      //             if(mpInitializer) delete mpInitializer;//useless

      mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      return;
    }
  } else {
    // Try to initialize
    if ((int)mCurrentFrame.mvKeys.size() <= 100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(nullptr);
      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches < 100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer*>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
      return;
    }

    cv::Mat Rcw;                  // Current Camera Rotation
    cv::Mat tcw;                  // Current Camera Translation
    vector<bool> vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i] = -1;
          nmatches--;
        }
      }

      // Set Frame Poses
      mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
      cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      tcw.copyTo(Tcw.rowRange(0, 3).col(3));
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB, true);
  KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, true,
                                  pKFini);  // zzh, it's very important! or assert failed in KF->SetBadFlag()

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0) continue;

    // Create MapPoint.
    cv::Mat worldPos(mvIniP3D[i]);

    MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame.AddMapPoint(pMP, mvIniMatches[i]);
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    // Add to Map
    mpMap->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  // Bundle Adjustment
  PRINT_INFO_MUTEX("New Map created with " << mpMap->MapPointsInMap() << " points" << endl);

  Optimizer::GlobalBundleAdjustment(mpMap, 20);

  // Set median depth to 1
  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth = 1.0f / medianDepth;

  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
    PRINT_INFO_MUTEX("Wrong initialization, reseting..." << endl);
    Reset();
    return;
  }

  // Scale initial baseline
  cv::Mat Tc2w = pKFcur->GetPose();
  Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      MapPoint* pMP = vpAllMapPoints[iMP];
      pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
    }
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFcur;  // next 2KFs' ComputePreInt()/Preintegration(2) need this variable set to current KF
  mCurrentFrame.mpReferenceKF = pKFcur;

  // zzh, initial KF doesn't need to ComputePreInt(), and PreIntegration()/Clear IMUData buffer before initial KF is
  // done in MonocularInitialization()
  plast_kf_ = pKFini;
  // zzh, this will automatically Clear IMUData buffer before mCurrentFrame/pKFcur, must be put after Wrong return
  PreIntegration(2);
  plast_kf_ = pKFcur;

  mLastFrame = Frame(mCurrentFrame, true);

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

#ifndef MUTE_VIEWER
  mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());
#endif

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
  // Set of MapPoints already found in the KeyFrame
  const auto& lfmps = mLastFrame.GetMapPointMatches();
#ifndef CHECK_REPLACE_ALL
  set<pair<MapPoint*, size_t>> spAlreadyFound = mLastFrame.GetMapPointsCami();
#endif
  size_t num_pts_replaced = 0;
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPoint* pMP = lfmps[i];

    if (pMP) {
      MapPoint* pRep = pMP->GetReplaced();
      if (pRep) {
#ifdef USE_SIMPLE_REPLACE
        if (pRep) mLastFrame.ReplaceMapPointMatch(i, pRep);
        continue;
#endif
        size_t cami = mLastFrame.mapn2in_.size() <= i ? 0 : get<0>(mLastFrame.mapn2in_[i]);
        CV_Assert(pMP->isBad());
        // not wise to search replaced too deep if this replace is outlier or max_depth too large
        int depth = 0, depth_thresh = 5;
        while (pRep && pRep->isBad()) {
#ifndef CHECK_REPLACE_ALL
          // solve0: lastframe seen mp-1->mp0->mp1, mp0->mp1, (mp0 seen in lastframe), then mp-1 should be directly
          // erased
          if (spAlreadyFound.end() != spAlreadyFound.find(make_pair(pRep, cami))) break;
#endif
          pRep = pRep->GetReplaced();
          if (++depth >= depth_thresh) break;
        }
        // notice that 2features in the same frame could see the same mp, caused by replace op.
        bool bcontinue = false;
        // solve1: lastframe seen mp-1->mp0->mp1->null, (mp1 bad), then mp-1 should be directly erased
        if (!pRep || pRep->isBad()
#ifndef CHECK_REPLACE_ALL
            || spAlreadyFound.end() != spAlreadyFound.find(make_pair(pRep, cami))  // solve0
#endif
        ) {
          bcontinue = true;
        }
#ifndef CHECK_REPLACE_ALL
        else if (mnLastKeyFrameId == mLastFrame.mnId) {
          // solve2: lba curkf/vpMapPoints1 4961->607 in pKF(18) then 4959->607 in pKF(19)
          if (!pRep->IsInKeyFrame(plast_kf_, i, cami)) bcontinue = true;
        }
        /*
         Notice that multithread bugs may still cause multi features in one cam to be matched to the same map point\
         cannot solve3: lba curkf 4961->607, 4959->608, code run here(4961->607), then lba thread fuse 607->608,
         here(4959->608), next frame tracking will fuse 4961matchidx->608\ cannot solve4: lba curkf 4961->608,
         4959->609, code run here(4961->608,4959->609), then lba thread fuse 608->609/608->610+609->610, next frame
         tracking will fuse 2fts to 1mp
         */
#endif
        if (bcontinue) {
          mLastFrame.EraseMapPointMatch(i);
          mLastFrame.mvbOutlier[i] = false;
          continue;
        }
        // cannot lock1 then lock2 while other thread lock2 then lock1!
        PRINT_DEBUG_FILE(i << "lfreplace" << pRep->mnId << ":bad" << (int)pRep->isBad(), mlog::vieo_slam_debug_path,
                         "tracking_thread_debug.txt");
#ifndef CHECK_REPLACE_ALL
        if (spAlreadyFound.end() != spAlreadyFound.find(make_pair(pRep, cami)))
          PRINT_DEBUG_FILE(endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
          //        CV_Assert(spAlreadyFound.end() == spAlreadyFound.find(pRep));
#endif
        mLastFrame.ReplaceMapPointMatch(i, pRep);  // if it's replaced by localMapper, use the replaced one
                                                   //        spAlreadyFound.insert(pRep);
        ++num_pts_replaced;
      }
    }
  }
  PRINT_DEBUG_FILE("replace num=" << num_pts_replaced << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
}

bool Tracking::TrackReferenceKeyFrame(int thInMPs, int thMatch) {
  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.7, true);  // stricter then SBP in TrackLocalMap()
  vector<MapPoint*> vpMapPointMatches;

  int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame,
                                     vpMapPointMatches);  // match with rKF,rectify the vpMapPointMatches, SBBoW better
                                                          // than SBP for the mCurrentFrame.Tcw is unknown
  PRINT_DEBUG_FILE("math num=" << nmatches << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  if (nmatches < thMatch)  // 15)//looser than 20 in TrackWithMotionModel()
    return false;

  mCurrentFrame.GetMapPointsRef() =
      vpMapPointMatches;                          // use temporary vector<MapPoint*> for not believe SBBow() so much
  mCurrentFrame.SetPose(mLastFrame.GetTcwRef());  // but use lF as the initial value for BA

  int num_inliers = Optimizer::PoseOptimization(&mCurrentFrame);  // motion-only BA
  PRINT_DEBUG_FILE("inliers=" << num_inliers << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  // Discard outliers
  int nmatchesMap = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (curfmps[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = curfmps[i];  // use temporary pointer to avoid mutex problem?

        mCurrentFrame.EraseMapPointMatch(i);
#ifndef NO_TRACK_MAP
        mCurrentFrame.mvbOutlier[i] = false;
#endif
        auto& trackinfo = pMP->GetTrackInfoRef();
        trackinfo.Reset(&mCurrentFrame);
        nmatches--;                               // useless here
      } else if (curfmps[i]->Observations() > 0)  // where EraseObservation()/SetBadFlag()?
        nmatchesMap++;
    }
  }
  mnMatchesInliers = nmatchesMap;

  return nmatchesMap >= thInMPs;  // 10;//Track ok when enough inlier MapPoints
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFrame* pRef = mLastFrame.mpReferenceKF;
  cv::Mat Tlr = mlRelativeFramePoses.back();

  mLastFrame.SetPose(Tlr * pRef->GetPose());

  if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking) return;

  // similar with the part in CreateNewKeyFrame()
  //  Create "visual odometry" MapPoints
  //  We sort points according to their measured depth by the stereo/RGB-D sensor
  vector<pair<float, int>> vDepthIdx;
  vDepthIdx.reserve(mLastFrame.N);
  for (int i = 0; i < mLastFrame.N; i++) {
    float z = mLastFrame.stereoinfo_.vdepth_[i];
    if (z > 0) {
      vDepthIdx.push_back(make_pair(z, i));
    }
  }

  if (vDepthIdx.empty()) return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth)
  // If less than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  const auto& lfmps = mLastFrame.GetMapPointMatches();
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;
    bool bcreatenew = false;
    MapPoint* pmp_old = lfmps[i];
    if (!pmp_old || pmp_old->Observations() < 1) bcreatenew = true;

    if (bcreatenew) {
      cv::Mat x3D = mLastFrame.UnprojectStereo(i);
      MapPoint* pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);  // different here, TODO: unify it

      size_t ididxs = mLastFrame.GetMapn2idxs(i);
      if (-1 == ididxs) {
        mLastFrame.AddMapPoint(pNewMP, i);
      } else {
        auto idxs = mLastFrame.mvidxsMatches[ididxs];
        bool icheck = false;
        for (int cami = 0; cami < idxs.size(); ++cami) {
          if (-1 != idxs[cami]) {
            auto icami = mLastFrame.mapin2n_[cami][idxs[cami]];
            bcreatenew = false;
            if (icami == i) {
              icheck = true;
              bcreatenew = true;
            } else {
              pmp_old = lfmps[icami];
              if (!pmp_old || pmp_old->Observations() < 1) bcreatenew = true;
            }
            if (bcreatenew) {
              mLastFrame.AddMapPoint(pNewMP, icami);
            }
          }
        }
        assert(icheck);
      }

      mlpTemporalPoints.push_back(pNewMP);
      nPoints++;
    } else {
      nPoints++;  // can be optimzed here
    }

    if (vDepthIdx[j].first > mThDepth && nPoints > 100) break;
  }
}

bool Tracking::TrackWithMotionModel() {
  ORBmatcher matcher(0.9, true);  // here 0.9 is useless

  // Update last frame pose according to its reference keyframe for rKF may be rectified
  // Create "visual odometry" points if in Localization Mode

  UpdateLastFrame();

  mCurrentFrame.SetPose(mVelocity * mLastFrame.GetTcwRef());  // Tc2c1*Tc1w

  // fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//already
  // initialized in Frame constructor if this Track function is firstly called

  // Project points seen in previous frame
  int th;
  if (mSensor != System::STEREO)
    th = 15;
  else
    th = 7;
  // has CurrentFrame.mvpMapPoints[bestIdx2]=pMP; in this func. then it can use m-o BA
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);
  PRINT_DEBUG_FILE("math num2=" << nmatches << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  // If few matches, uses a wider window search
  if (nmatches < 20) {
    auto& curfmps_ref = mCurrentFrame.GetMapPointsRef();
    // it's important for SBP() will not rectify the alreay nice CurretFrame.mvpMapPoints
    fill(curfmps_ref.begin(), curfmps_ref.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
  }

  if (nmatches < 20) return false;

#ifdef CHECK_JITTER
  mCurrentFrame.UpdateNavStatePVRFromTcw();
  NavState oldns = mCurrentFrame.GetNavState();
#endif
  // Optimize frame pose with all matches
  int num_inliers = Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame);  // motion-only BA
  //     Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA
  PRINT_DEBUG_FILE("inliers2=" << num_inliers << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
#ifdef CHECK_JITTER
  using Tdata = double;
  NavStated newns = mCurrentFrame.GetNavState();
  Sophus::SO3ex<Tdata> diff_rot = newns.mRwb * oldns.mRwb.inverse();
  Sophus::Vector3<Tdata> diff_p = newns.mpwb - oldns.mpwb;
  Sophus::Vector3<Tdata> diff_v = newns.mvwb - oldns.mvwb;
  bool identical = true;
  if (diff_rot.log().norm() > 0.2 || diff_p.norm() > 0.1 || diff_v.norm() > 1.5) {
    identical = false;
  }
  if (!identical) CV_Assert(0);
#endif

  // Discard outliers
  int nmatchesMap = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (curfmps[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint* pMP = curfmps[i];

        mCurrentFrame.EraseMapPointMatch(i);
#ifndef NO_TRACK_MAP
        mCurrentFrame.mvbOutlier[i] = false;
#endif
        auto& trackinfo = pMP->GetTrackInfoRef();
        trackinfo.Reset(&mCurrentFrame);
        nmatches--;
      } else if (curfmps[i]->Observations() > 0)
        nmatchesMap++;
    }
  }
  mnMatchesInliers = nmatchesMap;
  //  cout << "check matchinliers=" << mnMatchesInliers << endl;

  if (mbOnlyTracking) {
    // change to VO mode if the inlier MapPoint is too few i.e. robot goes to the new environment outside of the given
    // map
    mbVO = nmatchesMap < 10;
    return nmatches > 20;  // Track ok when enough inlier matches
  }

  return nmatchesMap >= 10;  // Track ok when enough inlier MapPoints
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();

  PRINT_DEBUG_FILE_MUTEX("SLP" << endl, mlog::vieo_slam_debug_path, "debug.txt");
  SearchLocalPoints();

#ifdef CHECK_JITTER
  mCurrentFrame.UpdateNavStatePVRFromTcw();
  NavState oldns = mCurrentFrame.GetNavState();
#endif
  // Optimize Pose
  // motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();
  int num_inliers = Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame);
  // motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();
  //      Optimizer::PoseOptimization(&mCurrentFrame);
#ifdef CHECK_JITTER
  using Tdata = double;
  NavStated newns = mCurrentFrame.GetNavState();
  Sophus::SO3ex<Tdata> diff_rot = newns.mRwb * oldns.mRwb.inverse();
  Sophus::Vector3<Tdata> diff_p = newns.mpwb - oldns.mpwb;
  Sophus::Vector3<Tdata> diff_v = newns.mvwb - oldns.mvwb;
  bool identical = true;
  if (diff_rot.log().norm() > 0.2 || diff_p.norm() > 0.1 || diff_v.norm() > 1.5) {
    identical = false;
  }
  if (!identical) CV_Assert(0);
#endif

  mnMatchesInliers = 0;
  PRINT_DEBUG_FILE("num_inliers22=" << num_inliers << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  // Update MapPoints Statistics
  set<MapPoint*> sMP;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for (int i = 0; i < mCurrentFrame.N; i++) {
    auto& pMP = curfmps[i];
    if (pMP) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        /*if (sMP.count(pMP))
          continue;
        else
          sMP.insert(pMP);*/
        pMP->IncreaseFound();
        if (!mbOnlyTracking) {
          if (pMP->Observations() > 0) mnMatchesInliers++;
        } else
          mnMatchesInliers++;
      } else {
        // why not include System::RGBD?maybe or RGBD lba thread can do faster.
#ifdef ORB3_STRATEGY_KF_MORE
        if (mSensor == System::STEREO && !mpIMUInitiator->GetVINSInited()) mCurrentFrame.EraseMapPointMatch(i);
#else
        if (mSensor == System::STEREO) mCurrentFrame.EraseMapPointMatch(i);
#endif
      }
    }
  }

  PRINT_DEBUG_FILE("inliers_map=" << mnMatchesInliers << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently (recent 1s)
  int threInlierReloc = 50, threInliers = 30;
  if (mbOnlyTracking &&
      mCurrentFrame.GetEncPreInt().mdeltatij >
          0) {  // rectified like TrackLocalMapWithIMU() for fusion effect (it largely affects Localization Mode)
    threInlierReloc = 25;
    threInliers = 15;
  }
  threInliers = 15;  // TODO: check
  if (mnLastRelocFrameId && mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < threInlierReloc)
    return false;

  if (mnMatchesInliers < threInliers)  // notice it's a class data member
    return false;
  else
    return true;
}

bool Tracking::NeedNewKeyFrame() {
  //#define ORB3_STRATEGY
  //#ifdef ORB3_STRATEGY
  //  if (mpIMUInitiator->GetSensorIMU() && !mpIMUInitiator->GetVINSInited()) {
  //    if (mCurrentFrame.ftimestamp_ - plast_kf_->ftimestamp_ >= 0.25)
  //      return true;
  //    else
  //      return false;
  //  }
  //#endif

  if (mbOnlyTracking) return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) return false;

  const int nKFs = mpMap->KeyFramesInMap();

  // Do not insert keyframes if not enough frames have passed from last relocalisation
  // the settings fps used here, if at initial step add new KF quickly while at relocalisation step add it slowly
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames) return false;

  // Do not insert keyframes if bias is not computed in VINS mode, maybe we can change localBA to pure-vision when
  // mbRelocBiasPrepare=true and create a thread to RecomputeIMUBiasAndCurrentNavstate() like IMU Initialization!
  if (mbRelocBiasPrepare) return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  // just check for one(with ur>=0) KF demand for RGBD, if one of former 2 KFs is ODOMOK must also use this like
  // nKFs<=2!!!
  if (nKFs <= 2 || mState == OK && plast_kf_->mnId < mnLastOdomKFId + 2) nMinObs = 2;
  // the number of good MinObs(for Monocular) KFs tracked MapPoints in the RefKF
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // check if Local Mapping accept keyframes or LM thread is idle
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Check how many "close" points are being tracked and how many could be potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;
  if (mSensor != System::MONOCULAR) {
    const auto& curfmps = mCurrentFrame.GetMapPointMatches();
    if (mCurrentFrame.stereoinfo_.v3dpoints_.empty()) {
      for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.stereoinfo_.vdepth_[i] > 0 &&
            mCurrentFrame.stereoinfo_.vdepth_[i] < mThDepth)  // it's a close point
        {
          if (curfmps[i] && !mCurrentFrame.mvbOutlier[i])  // it's a inlier map point or tracked one
            nTrackedClose++;
          else
            nNonTrackedClose++;
        }
      }
    } else {
      for (int k = 0; k < mCurrentFrame.stereoinfo_.v3dpoints_.size(); ++k) {
        if (mCurrentFrame.stereoinfo_.goodmatches_[k]) {
          size_t i = mCurrentFrame.mapidxs2n_[k];
          CV_Assert(-1 != i);
          float z = mCurrentFrame.stereoinfo_.vdepth_[i];
          CV_Assert(z > 0);
          if (z < mThDepth) {
            // it's a inlier map point or tracked one
            if (curfmps[i] && !mCurrentFrame.mvbOutlier[i])
              nTrackedClose++;
            else
              nNonTrackedClose++;
          }
        }
      }
    }
  }

  //τt=100(enough dis)( τc=70(enough info) for stereo/RGBD to insert a new KF
  bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

  double timegap = 0.5;
  bool bimu_inited = mpIMUInitiator->GetVINSInited();
  // JW uses different timegap during IMU Initialization(0.1s); 0.25 ref from ORB3
  if (!bimu_inited) timegap = 0.25;
  bool cTimeGap = false;
  int minClose = 70;
  bool bctimegap_judge = mpIMUInitiator->GetSensorIMU() || (mState == ODOMOK && System::MONOCULAR == mSensor);
  if (bctimegap_judge) {
    //#ifdef ORB3_STRATEGY
    //    if (!mpIMUInitiator->GetSensorEnc()) {
    //      // ref from ORB3
    //      cTimeGap = ((mCurrentFrame.ftimestamp_ - plast_kf_->ftimestamp_) >= timegap);
    //    } else
    //#endif
    {
      cTimeGap = ((mCurrentFrame.ftimestamp_ - plast_kf_->ftimestamp_) >= timegap) && bLocalMappingIdle &&
                 mnMatchesInliers > 15;
      // if (bimu_inited){//also we can use GetSensorIMU()
      // we still need this for tum_vi outdoors2 ds
      // for VIO+Stereo/RGB-D, we don't open this inerstion strategy for speed and cTimeGap can do similar jobs
      // when ORB3_STRATEGY added, c1c unused but c2 extended for IMU_STEREO
      // bNeedToInsertClose = false;
      // for VIEO+RGB-D(Stereo)/VIO with RECENTLY_LOST, cTimeGap won't affect ODOMOK, so we may need it
      if (mState == ODOMOK) {
        cTimeGap = ((mCurrentFrame.ftimestamp_ - plast_kf_->ftimestamp_) >= timegap) && bLocalMappingIdle;
      }
      // minClose=100;
    }
  }

  // Thresholds
  float thRefRatio = 0.75f;
  // it's necessary for this stricter enough distance threshold! in my dataset Corridor004, like nKFs<=2!
  if (nKFs <= 2 || (mState == OK && plast_kf_->mnId < mnLastOdomKFId + 2)) thRefRatio = 0.4f;

  if (mSensor == System::MONOCULAR) thRefRatio = 0.9f;  // JingWang uses 0.8f

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  // time span too long(1s)
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  // for minF=0, if LocalMapper is idle
  const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
  // Condition 1c: tracking is weak
  // rgbd/stereo tracking weak outside(large part are far points)
#ifdef ORB3_STRATEGY_KF_MORE
  // for Mono/IMU_STREREO won't erase Frame's pMP match, where c1c can easily enter and cause lba thread overload
  const bool c1c =
      mSensor != System::MONOCULAR && !bimu_inited && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
#else
  // for Mono won't erase Frame's pMP match, where c1c can easily enter and cause lba thread overload
  const bool c1c =
      mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || (!bimu_inited && bNeedToInsertClose));
#endif
  // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  // not too close && not too far
  const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

  // Condition 3: odom && time conditon && min new close points' demand
  // may we can also use &&mCurrentFrame.N>500 like StereoInitialization(), so c3 now is not for VIO Mono RECENTLY_LOST
  const bool c3 = (mState == ODOMOK) && (c1a || c1b || c1c) && nNonTrackedClose > 70;

  if (((c1a || c1b || c1c) && c2) || cTimeGap || c3)  // cTimeGap added by JingWang
  {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      if (mSensor != System::MONOCULAR) {
        if (mpLocalMapper->KeyframesInQueue() < 3)
          return true;
        else  // it wating queue is too long still refuse to insert new KFs
          return false;
      }  // it it's monocular, LocalMapper not idle -> refuse to insert new KFs
      else
        return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame(vector<cv::Mat> imgs_dense) {
  // if localMapper is stopped by loop closing thread/GUI, cannot add KFs; during adding process, it cannot be stopped
  // by others
  if (!mpLocalMapper->SetNotStop(true)) return;

  // ensure Tcw is always right for mCurrentFrame even there's no odom data, NavState/Tbw can be wrong when there's no
  // odom data
  // copy initial Tcw&Tbw(even wrong), update bi=bi+dbi
  KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, true, plast_kf_, mState);
  // here notice a fact: JingWang haven't considered no imu data's condition when imu is initialized(including reloc.),
  // so his NavState after imu intialized is always right, but before is also wrong, but he should set the right
  // NavState for the first initialized KeyFrame so I need to UpdateNavStatePVRFromTcw for the Frame when imu data is
  // empty after imu is initialized
  if (mState == ODOMOK) mnLastOdomKFId = pKF->mnId;

  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  PreIntegration(2);  // zzh, though it doesn't need to be calculated when IMU isn't initialized

  if (mSensor != System::MONOCULAR) {
    pKF->imgs_dense_ = imgs_dense;  // zzh for PCL map creation

    mCurrentFrame.UpdatePoseMatrices();  // UnprojectStereo() use mRwc,mOw, maybe useless

    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest close points
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mCurrentFrame.N);
    if (mCurrentFrame.stereoinfo_.v3dpoints_.empty()) {
      for (int i = 0; i < mCurrentFrame.N; i++) {
        float z = mCurrentFrame.stereoinfo_.vdepth_[i];
        if (z > 0) {
          vDepthIdx.push_back(make_pair(z, i));
        }
      }
    } else {  // TODO: speed up code, which could be unified with that in NeedNewKeyFrame()
      for (int k = 0; k < mCurrentFrame.stereoinfo_.v3dpoints_.size(); ++k) {
        if (mCurrentFrame.stereoinfo_.goodmatches_[k]) {
          int i = mCurrentFrame.mapidxs2n_[k];
          float z = mCurrentFrame.stereoinfo_.vdepth_[i];
          CV_Assert(-1 != i && z > 0);
          vDepthIdx.push_back(make_pair(z, i));
        }
      }
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      const auto& curfmps = mCurrentFrame.GetMapPointMatches();
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        size_t ididxs = mCurrentFrame.GetMapn2idxs(i);
        vector<MapPoint*> pmps;
        if (-1 == ididxs) {
          MapPoint* pMP = curfmps[i];
          if (!pMP)
            bCreateNew = true;
          else if (pMP->Observations() < 1) {
            bCreateNew = true;
            assert(mbOnlyTracking || pMP->isBad());
            // is there memory leak? we could use shared_ptr to solve this problem
            mCurrentFrame.EraseMapPointMatch(i);
          }
          pmps.push_back(pMP);
        } else {
          bCreateNew = true;
          auto idxs = mCurrentFrame.mvidxsMatches[ididxs];
          bool icheck = false;
          for (int cami = 0; cami < idxs.size(); ++cami) {
            if (-1 != idxs[cami]) {
              auto icami = mCurrentFrame.mapin2n_[cami][idxs[cami]];
              if (icami == i) icheck = true;
              MapPoint* pMP = curfmps[icami];
              // old strategy won't abandon all bad mps, only obs<1 bad mps will be substituted by stereo pt
              if (pMP && !pMP->isBad()) {  // we don't replace old good mps with stereo pt
                bCreateNew = false;
                // break; //use this when no pmps check
              }
              pmps.push_back(pMP);
            }
          }
        }

        if (bCreateNew) {
          cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
          MapPoint* pNewMP = new MapPoint(x3D, pKF, mpMap);
          size_t ididxs = mCurrentFrame.GetMapn2idxs(i);
          PRINT_DEBUG_FILE_MUTEX("mp0[" << pKF->mnId << "," << i << "]:", mlog::vieo_slam_debug_path, "debug.txt");
          if (-1 == ididxs) {
            pNewMP->AddObservation(pKF, i);
            pKF->AddMapPoint(pNewMP, i);
            mCurrentFrame.AddMapPoint(pNewMP, i);
          } else {
            auto idxs = mCurrentFrame.mvidxsMatches[ididxs];
            bool icheck = false;
            for (int cami = 0; cami < idxs.size(); ++cami) {
              if (-1 != idxs[cami]) {
                auto icami = mCurrentFrame.mapin2n_[cami][idxs[cami]];
                if (icami == i) icheck = true;
                pNewMP->AddObservation(pKF, icami);
                pKF->AddMapPoint(pNewMP, icami);
                mCurrentFrame.AddMapPoint(pNewMP, icami);
              }
            }
            PRINT_DEBUG_FILE_MUTEX(endl, mlog::vieo_slam_debug_path, "debug.txt");
            assert(icheck);
          }
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpMap->AddMapPoint(pNewMP);
          nPoints++;
        } else {
          nPoints++;
        }

        // maybe we can also use this for ODOMOK like StereoInitialization()
        if (vDepthIdx[j].first > mThDepth && nPoints > 100)  //&&mState==OK)
          break;
      }
    }
  }

#ifndef NO_LBA_THREAD
  mpLocalMapper->InsertKeyFrame(pKF);
#endif
  PRINT_DEBUG_FILE("curf is kf" << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  plast_kf_ = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched (in TrackWithMotionModel()/...),
  // all of (these) map points created by CreateNewKeyFrame()/StereoInitialization()/{UpdateLastFrame()in Localization
  // mode/CreateNewMapPoints() in LocalMapping}
  size_t num_maps_ready = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for (size_t i = 0; i < curfmps.size(); ++i) {
    MapPoint* pMP = curfmps[i];
    if (pMP) {
      if (pMP->isBad()) {
        mCurrentFrame.EraseMapPointMatch(i);
      } else {
        pMP->IncreaseVisible();
        auto& trackinfo = pMP->GetTrackInfoRef();
        trackinfo.Reset(&mCurrentFrame);
        ++num_maps_ready;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend;
       vit++) {
    MapPoint* pMP = *vit;
    // jump the already in-mCurrentFrame.mvpMapPoints MapPoints
    if (pMP->GetTrackInfoRef().last_seen_frameid_ == mCurrentFrame.mnId) continue;
    if (pMP->isBad()) continue;
    // Project (this fills MapPoint variables for matching,like mbTrackInView=true...)
    // judge if mCurrentFrame's centre is in the effective descriptor area(scale&&rotation invariance) of the
    // MapPoint(with best descriptor&&normalVector)
    // no problem for mRcw,mtcw for mCurrentFrame.SetPose() in TrackWithMotionModel()/TrackReferenceKeyFrame()
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }
  PRINT_DEBUG_FILE("extra init=" << nToMatch << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8);  // 0.8 is the threshold for mindist/mindist2(<th is nice matching)
    int th = 1;
    if (mSensor == System::RGBD) th = 3;
    if (mpIMUInitiator->GetVINSInited()) {  // ref from ORB3
      bool bimu_stable = true;              // mpIMUInitiator->GetInitGBA2() && mpIMUInitiator->GetInitGBAOver();
      if (bimu_stable)
        th = 2;
      else
        th = 3;
    }

    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2) th = 5;

    if (ODOMOK == mState) th = 15;  // ref from ORB3

    // th=10; // ORB3 use 10 here for imu_stereo
    // rectify the mCurrentFrame.mvpMapPoints
    nToMatch = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->th_far_pts_);
  }
  PRINT_DEBUG_FILE("befopt2 extra=" << nToMatch << endl, mlog::vieo_slam_debug_path, "tracking_thread_debug.txt");
}

void Tracking::UpdateLocalMap() {
  // This is for visualization,but visualized MapPoints are the last F ones
  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    KeyFrame* pKF = *itKF;
    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for (vector<MapPoint*>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++) {
      MapPoint* pMP = *itMP;
      if (!pMP) continue;
      // current F visible MapPoints initial mnTrackReferenceForFrame==0(mCurrentFrame.mnId entering this func. cannot
      // be 0)
      if (pMP->GetTrackInfoRef().track_ref_frameid_ == mCurrentFrame.mnId) continue;
      if (!pMP->isBad()) {
        mvpLocalMapPoints.push_back(pMP);
        pMP->GetTrackInfoRef().track_ref_frameid_ = mCurrentFrame.mnId;  // so it's for avoiding redundant addition
      }
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame*, int> keyframeCounter;
  Frame* pref_f = &mCurrentFrame;
#ifdef ORB3_STRATEGY_TRACK_BA_ONCE
  if (mpIMUInitiator->GetVINSInited() && !mbRelocBiasPrepare) {
    pref_f = &mLastFrame;
  }
#endif
  const auto& curfmps = pref_f->GetMapPointMatches();
  for (int i = 0; i < pref_f->N; i++) {
    if (curfmps[i]) {
      MapPoint* pMP = curfmps[i];
      if (!pMP->isBad()) {
        const auto observations = pMP->GetObservations();
        for (map<KeyFrame*, set<size_t>>::const_iterator it = observations.begin(), itend = observations.end();
             it != itend; it++)
          keyframeCounter[it->first]++;
      } else {
        pref_f->EraseMapPointMatch(i);
      }
    }
  }

  if (keyframeCounter.empty()) return;

  int max = 0;
  KeyFrame* pKFmax = static_cast<KeyFrame*>(nullptr);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
  for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd;
       it++) {
    KeyFrame* pKF = it->first;

    if (pKF->isBad()) continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);              // looser than covisibility graph demand
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;  // to avoid repetition when selecting neighbor KFs(2nd layer
                                                         // covisible KFs&& neighbors of the spanning tree)
  }

  // Include also some not-already-included keyframes that are neighbors to already-included keyframes
  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) break;

    KeyFrame* pKF = *itKF;

    const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
#ifdef NO_LBA_THREAD
    CV_Assert(!vNeighs.size());
#endif

    for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame* pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        // avoid for replicated push_back for different itKF, this cannot be mCurrentFrame(not KF now)
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    const set<KeyFrame*> spChilds = pKF->GetChilds();
#ifdef NO_LBA_THREAD
    CV_Assert(!spChilds.size());
#endif
    for (set<KeyFrame*>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
      KeyFrame* pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame* pParent = pKF->GetParent();
#ifdef NO_LBA_THREAD
    CV_Assert(!pParent);
#endif
    if (pParent) {
      if (!pParent->isBad() && pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        // break;
      }
    }
  }

  // ref from ORB3
  // Add 10 last temporal KFs (mainly for IMU)
  if (mpIMUInitiator->GetVINSInited() && mvpLocalKeyFrames.size() < 80) {
    KeyFrame* tempKeyFrame = plast_kf_;

    const int Nd = 20;
    for (int i = 0; i < Nd; i++) {
      if (!tempKeyFrame) break;
      if (!tempKeyFrame->isBad() && tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(tempKeyFrame);
        tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        tempKeyFrame = tempKeyFrame->GetPrevKeyFrame();
      }
    }
  }

  if (pKFmax)  // still maybe rectify this two refKF variables in CreateNewKeyFrame()
  {
    mpReferenceKF = pKFmax;  // highest/max covisible weight/MPs KF
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
  vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

  if (vpCandidateKFs.empty()) return false;

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true);  // similar threshold in TrackReferenceKeyFrame()

  vector<PnPsolver*> vpPnPsolvers;
  vpPnPsolvers.resize(nKFs);

  vector<vector<MapPoint*>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame* pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
      if (nmatches < 15)  // same threshold in TrackReferenceKeyFrame()
      {
        vbDiscarded[i] = true;
        continue;  // useless
      } else {
        PnPsolver* pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);  // get transformation by 3D-2D
                                                                                   // matches
        pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
        vpPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  ORBmatcher matcher2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i]) continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      PnPsolver* pSolver = vpPnPsolvers[i];
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reaches max. iterations discard keyframe
      if (bNoMore)  // to avoid too long time in RANSAC, at most 300 iterations from while start here
      {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (!Tcw.empty()) {
        Tcw.copyTo(mCurrentFrame.GetTcwRef());

        set<MapPoint*> sFound;

        const int np = vbInliers.size();  // vvpMapPointMatches[i].size()/mCurrentFrame.mvpMapPoints.size()

        for (int j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame.ReplaceMapPointMatch(j, vvpMapPointMatches[i][j]);
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.EraseMapPointMatch(j);
        }

        // use RANSAC P4P to select good inlier 3D-2D matches, then use all these matches to motion-BA
        int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        // without checking pMP->Observation(), the number of inliers by motion-BA, the same threshold as
        // TrackWithMotionModel()/TrackReferenceKeyFrame()
        if (nGood < 10) continue;

        for (int io = 0; io < mCurrentFrame.N; io++)  // delete outliers
          if (mCurrentFrame.mvbOutlier[io]) mCurrentFrame.EraseMapPointMatch(io);

        // If few inliers, search by projection in a coarse or fine window and optimize again
        if (nGood < 50) {
          int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

          if (nadditional + nGood >= 50) {
            nGood = Optimizer::PoseOptimization(&mCurrentFrame);  // using additional matches to motion-BA

            // If many inliers but still not enough, search by projection again in a narrower window
            // the camera has been already optimized with many points
            if (nGood > 30 && nGood < 50) {
              sFound.clear();
              const auto& curfmps = mCurrentFrame.GetMapPointMatches();
              for (int ip = 0; ip < mCurrentFrame.N; ip++)
                if (curfmps[ip])  // include outliers of just former BA for nGood isn't changed or don't believe former
                                  // wide SBP() BA
                  sFound.insert(curfmps[ip]);
              nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

              // Final optimization
              if (nGood + nadditional >= 50) {
                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                for (int io = 0; io < mCurrentFrame.N; io++)
                  if (mCurrentFrame.mvbOutlier[io]) mCurrentFrame.EraseMapPointMatch(io);
              }
            }  // why don't delete mCurrentFrame.mvbOutlier when nGood>=50?
          }
        }

        // If the pose is supported by enough inliers stop ransacs and continue
        // the same threshold as TrackLocalMap() in Relocalization mode
        if (nGood >= 50) {
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    return false;
  } else {
    mnLastRelocFrameId = mCurrentFrame.mnId;

    // Tracking mode doesn't enter this part
    if (!mbOnlyTracking && !mpIMUInitiator->mbUsePureVision) {
      auto bsensor_imu = mpIMUInitiator->GetSensorIMU(), bimu_inited = mpIMUInitiator->GetVINSInited();
      if (bsensor_imu) {
        assert(mpIMUInitiator->GetVINSInited() && "VINS not inited? why.");
        // notice we should call RecomputeIMUBiasAndCurrentNavstate() when 20-1 frames later, see IV-E in VIORBSLAM
        // paper
        mbRelocBiasPrepare = true;
      } else if (bimu_inited) {
        // for vins inited set true, we should ensure reloc_prep mode but imu data must input(bsensor_imu)
        return false;
      }
    }
    return true;
  }
}

void Tracking::Reset() {
  PRINT_INFO_MUTEX("System Reseting" << endl);

  if (mpViewer) {
    mpViewer->RequestStop();
    while (!mpViewer->isStopped()) usleep(3000);
  }

  // Reset Local Mapping
  PRINT_INFO_MUTEX("Reseting Local Mapper...");
  mpLocalMapper->RequestReset();
  PRINT_INFO_MUTEX(" done" << endl);

  // Reset Loop Closing
  PRINT_INFO_MUTEX("Reseting Loop Closing...");
  mpLoopClosing->RequestReset();
  PRINT_INFO_MUTEX(" done" << endl);

  // zzh: Reset IMU Initialization, must after mpLocalMapper&mpLoopClosing->RequestReset()! for no updation of
  // mpCurrentKeyFrame& no use of mbVINSInited in IMUInitialization thread
  cout << "Resetting IMU Initiator...";
  mpIMUInitiator->RequestReset();
  cout << " done" << endl;

  mbRelocBiasPrepare = false;
  mnLastOdomKFId = 0;
  mnLastRelocFrameId = 0;
  SetInitLastKeyFrame(nullptr);  // for safety and LoadMap() SetInitLastKeyFrame
  SetInitReferenceKF(nullptr);   // for safety and LoadMap()

  // Clear BoW Database
  PRINT_INFO_MUTEX("Reseting Database...");
  mpKeyFrameDB->clear();
  PRINT_INFO_MUTEX(" done" << endl);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpMap->clear();

  MapPoint::nNextId = 0;  // added by zzh
  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;

  // for monocular!
  if (mpInitializer) {
    delete mpInitializer;
    mpInitializer = static_cast<Initializer*>(nullptr);
  }

  mlRelativeFramePoses.clear();
  relative_frame_bvwbs_.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();

  if (mpViewer) mpViewer->Release();
}

void Tracking::InformOnlyTracking(const bool& flag) {
  mbOnlyTracking = flag;
  // added by zzh
  // a switch for VIEO/VIO suitable tracking mode(VEO/RGBD) and VIEO/VIO 2nd/continuous SLAM mode
  static bool bSwitch = false;
  if (flag) {
    if (mpIMUInitiator->GetVINSInited()) {
      bSwitch = true;
      // notice tracking mode only has VO/VEO, so VIEO uses VEO & VIO uses VO
      mpIMUInitiator->SetVINSInited(false);
    }
  } else {
    if (bSwitch) {
      bSwitch = false;
      mpIMUInitiator->SetVINSInited(true);
      mState = LOST;
    }
  }
}

}  // namespace VIEO_SLAM
