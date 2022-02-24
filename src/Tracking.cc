/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/leavesnight/VIEO_SLAM>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "common/log.h"
#include "common/common.h"
#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "radtan.h"
#include "KannalaBrandt8.h"
#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

using namespace std;

namespace VIEO_SLAM
{

cv::Mat Tracking::CacheOdom(const double &timestamp, const double* odomdata, const char mode){//different thread from GrabImageX
  //you can add some odometry here for fast Tcw retrieve(e.g. 200Hz)
  unique_lock<mutex> lock(mMutexOdom);
  switch (mode){
    case System::ENCODER://only encoder
      if (mlOdomEnc.empty()){
        mlOdomEnc.push_back(EncData(odomdata,timestamp+mDelayToEnc));//notice Timg=Todom+delay
        miterLastEnc=mlOdomEnc.begin();
      }else
	    mlOdomEnc.push_back(EncData(odomdata,timestamp+mDelayToEnc));
      mpIMUInitiator->SetSensorEnc(true);
      break;
    case System::IMU://only q/awIMU
      if (mlOdomIMU.empty()){
	    mlOdomIMU.push_back(IMUData(odomdata,timestamp+mDelayToIMU));
	    miterLastIMU=mlOdomIMU.begin();
      }else
	    mlOdomIMU.push_back(IMUData(odomdata,timestamp+mDelayToIMU));
#ifndef TRACK_WITH_IMU
      ;//mbSensorIMU=false;
#else
      mpIMUInitiator->SetSensorIMU(true);
#endif
      break;
    case System::BOTH://both encoder & q/awIMU
      if (mlOdomEnc.empty()){
        mlOdomEnc.push_back(EncData(odomdata,timestamp+mDelayToEnc));
        miterLastEnc=mlOdomEnc.begin();
      }else
	    mlOdomEnc.push_back(EncData(odomdata,timestamp+mDelayToEnc));
      if (mlOdomIMU.empty()){
	    mlOdomIMU.push_back(IMUData(odomdata+2,timestamp+mDelayToIMU));
	    miterLastIMU=mlOdomIMU.begin();
      }else
	    mlOdomIMU.push_back(IMUData(odomdata+2,timestamp+mDelayToIMU));
      mpIMUInitiator->SetSensorEnc(true);
#ifndef TRACK_WITH_IMU
      ;//mbSensorIMU=false;
#else
      mpIMUInitiator->SetSensorIMU(true);
#endif
      break;
  }

  return cv::Mat();
}

void Tracking::TrackWithOnlyOdom(bool bMapUpdated){
  if (!mpIMUInitiator->GetVINSInited()){//VEO, we use mVelocity as EncPreIntegrator from LastFrame if EncPreIntegrator exists
    assert(!mVelocity.empty());
    cv::Mat Tcw=mVelocity*mLastFrame.GetTcwRef();//To avoid accumulated numerical error of pure encoder predictions causing Rcw is not Unit Lie Group (|Rcw|=1)!!!
    Matrix3d eigRcw=Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
    cv::Mat Rcw=Converter::toCvMat(Sophus::SO3exd::normalizeRotationM(eigRcw));
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    mCurrentFrame.SetPose(Tcw);//if directly use mVelocity*mLastFrame.mTcw), will cause big bug when insertion strategy tends to less insertion!
    //it's difficult to get mCurrentFrame.mvbOutlier like motion-only BA
    mState=ODOMOK;
    cout<<greenSTR<<mVelocity.at<float>(0,3)<<" "<<mVelocity.at<float>(1,3)<<" "<<mVelocity.at<float>(2,3)<<whiteSTR<<endl;
    cout<<"ODOM KF: "<<mCurrentFrame.mTimeStamp<<endl;
  }else{//VIEO, for convenience, we don't use mVelocity as it won't be updated as EncPreIntegrator from LastFrame before
    using namespace Eigen;
    //motion update/prediction by Enc motion model
    const EncPreIntegrator &encpreint=mCurrentFrame.GetEncPreInt();
    //get To1o2:p12 R12
    Vector3d pij(encpreint.mdelxEij.segment<3>(3));
    Matrix3d Rij=Sophus::SO3exd::Expmap(encpreint.mdelxEij.segment<3>(0));
    cv::Mat Tij=Converter::toCvSE3(Rij,pij);
    //get Tc2c1
    cv::Mat Tec=Converter::toCvMatInverse(Frame::mTce);
    cv::Mat TlwIE=bMapUpdated?mpLastKeyFrame->GetPose():mLastFrame.GetTcwRef();
    cv::Mat Tcw=Frame::mTce*Converter::toCvMatInverse(Tij)*Tec*TlwIE;//To avoid accumulated numerical error of pure encoder predictions
    Matrix3d eigRcw=Converter::toMatrix3d(Tcw.rowRange(0,3).colRange(0,3));
    cv::Mat Rcw=Converter::toCvMat(Sophus::SO3exd::normalizeRotationM(eigRcw));
    Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    mCurrentFrame.SetPose(Tcw);
    mCurrentFrame.UpdateNavStatePVRFromTcw();
    mState=ODOMOK;/*
    if (mCurrentFrame.GetIMUPreInt().mdeltatij>0){
      // Pose optimization. false: no need to compute marginalized for current Frame(motion-only), see VIORBSLAM paper (4)~(8)
      if(bMapUpdated){//we call this 2 frames'(FKF/FF) motion-only BA
	//not use Hessian matrix, it's ok
	Optimizer::PoseOptimization(&mCurrentFrame,mpLastKeyFrame,mpIMUInitiator->GetGravityVec(),true,true);//fixing lastKF(i), optimize curF(j), save its Hessian
      }else{
	//unfix lastF(j): Hessian matrix exists, use prior Hessian to keep lastF(j)'s Pose stable, optimize j&j+1; fix lastF(j): optimize curF(j+1)
	Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame,mpIMUInitiator->GetGravityVec(),true,true);//last F unfixed/fixed when lastF.mOdomPreIntIMU.deltatij==0 or RecomputeIMUBiasAndCurrentNavstate(), save its Hessian
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
    // Update motion model, we don't need to adjust mLastFrame.mTcw.empty() for before ODOMOK, it cannot be LOST(when LOST, PreIntegration(1/3) is not executed/mdeltatij==0, but (2) may be executed if it's a new KF)
    cv::Mat LastTwc=cv::Mat::eye(4,4,CV_32F);
    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
    mVelocity=mCurrentFrame.GetTcwRef()*LastTwc;//Tc2c1/Tcl
    cout<<greenSTR<<"IEODOM KF: "<<mCurrentFrame.mTimeStamp<<whiteSTR<<endl;
  }
}


void Tracking::PreIntegration(const int8_t type){
  unique_lock<mutex> lock(mMutexOdom);
  PRINT_DEBUG_INFO_MUTEX("type="<<(int)type<<"...", imu_tightly_debug_path, "debug.txt");
  FrameBase* plastfb, *pcurfb;
  if (type == 1 || type == 3) {
    if (type == 1)
      plastfb = static_cast<FrameBase*>(&mLastFrame);
    else
      plastfb = static_cast<FrameBase*>(mpLastKeyFrame);
    pcurfb = static_cast<FrameBase*>(&mCurrentFrame);
    bool bpreint = PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, mpLastKeyFrame, &mLastFrame,
                                           blast_kf2kfpreint_);
    if (!bpreint)
      brecompute_kf2kfpreint_[0] = true;
    else if (blast_kf2kfpreint_)
      brecompute_kf2kfpreint_[0] = false;
    //   cout<<"!"<<mlOdomIMU.size()<<endl;
    //   cout<<"encdata over"<<endl;
    bpreint = PreIntegration<IMUData>(type, mlOdomIMU, miterLastIMU, plastfb, pcurfb, mpLastKeyFrame, &mLastFrame,
                                      blast_kf2kfpreint_);
    if (!bpreint)
      brecompute_kf2kfpreint_[1] = true;
    else if (blast_kf2kfpreint_)
      brecompute_kf2kfpreint_[1] = false;
    blast_kf2kfpreint_ = false;
    //   cout<<"over"<<endl;
  } else {
    plastfb = static_cast<FrameBase*>(mpLastKeyFrame);
    pcurfb = type == 2 ? static_cast<FrameBase*>(mpReferenceKF) : static_cast<FrameBase*>(&mCurrentFrame);
    KeyFrame* plastkf = brecompute_kf2kfpreint_[0] ? nullptr : mpLastKeyFrame;
    PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, plastkf);
    plastkf = brecompute_kf2kfpreint_[1] ? nullptr : mpLastKeyFrame;
    PreIntegration<IMUData>(type, mlOdomIMU, miterLastIMU, plastfb, pcurfb, plastkf);
    // won't care initial value of this and how flow strategy it's (like pure vision then visualimu/mixed one)
    for (auto& brecompute : brecompute_kf2kfpreint_) brecompute = true;
    blast_kf2kfpreint_ = true;
  }
  if (type==2){
    size_t N=mpReferenceKF->GetListIMUData().size(),N2=mpReferenceKF->GetListEncData().size();
    PRINT_DEBUG_INFO_MUTEX("List size: "<<N<<" "<<N2<<endl, imu_tightly_debug_path, "debug.txt");
  }
}
bool Tracking::GetVelocityByEnc(bool bMapUpdated) {
  char type = 1; // if Map updated, still optimize with last frame
  {
    unique_lock<mutex> lock(mMutexOdom);
    FrameBase *plastfb, *pcurfb;
    plastfb = static_cast<FrameBase*>(&mLastFrame);
    pcurfb = static_cast<FrameBase*>(&mCurrentFrame);
    bool bpreint = PreIntegration<EncData>(type, mlOdomEnc, miterLastEnc, plastfb, pcurfb, mpLastKeyFrame, &mLastFrame,
                                           blast_kf2kfpreint_);
    if (!bpreint)
      brecompute_kf2kfpreint_[0] = true;
    else if (blast_kf2kfpreint_)
      brecompute_kf2kfpreint_[0] = false;
    blast_kf2kfpreint_ = false;
  }
  if (mCurrentFrame.GetEncPreInt().mdeltatij == 0) {
    return false;  // check PreIntegration() failed when mdeltatij==0, so mCurrentFrame.mTcw==cv::Mat()
  }

  using namespace Eigen;
  // motion update/prediction by Enc motion model
  const EncPreIntegrator& encpreint = mCurrentFrame.GetEncPreInt();  // problem exits
  // get To1o2:p12 R12
  Vector3d pij(encpreint.mdelxEij.segment<3>(3));
  Matrix3d Rij = Sophus::SO3exd::Expmap(encpreint.mdelxEij.segment<3>(0));
  cv::Mat Tij = Converter::toCvSE3(Rij, pij);
  // get Tc2c1
  cv::Mat Tec = Converter::toCvMatInverse(Frame::mTce);
  mVelocity = Frame::mTce * Converter::toCvMatInverse(Tij) * Tec;
  //   Matrix3d eigRcl=Converter::toMatrix3d(mVelocity.rowRange(0,3).colRange(0,3));//we don't need this correction for this numerical error caused from double to float won't be accumulated! cv::Mat Rcl=Converter::toCvMat(Sophus::SO3exd::normalizeRotationM(eigRcl));
  //   Rcl.copyTo(mVelocity.rowRange(0,3).colRange(0,3));

  //   cout<<encpreint.mdelxEij[0]<<endl;
  //   cout<<mVelocity.at<float>(0,3)<<" "<<mVelocity.at<float>(1,3)<<" "<<mVelocity.at<float>(2,3)<<endl;
  return true;
}
bool Tracking::TrackWithIMU(bool bMapUpdated){
    ORBmatcher matcher(0.9,true);//here 0.9 is useless

    // Update current frame pose according to last frame or keyframe when last KF is changed by LocalMapping/LoopClosing threads
    // unadded code: Create "visual odometry" points if in Localization Mode
    if (!PredictNavStateByIMU(bMapUpdated)) {
        PRINT_INFO_MUTEX( redSTR"CurF.mdeltatij==0! we have to TrackWithMotionModel()!" << whiteSTR << endl);
        if (!mVelocity.empty()) {
            bool bOK = TrackWithMotionModel();//maybe track failed then call pure-vision TrackWithMotionModel
            if (bOK)
                mCurrentFrame.UpdateNavStatePVRFromTcw();//don't forget to update NavState though bg(bg_bar,dbg)/ba(ba_bar,dba) cannot be updated
            return bOK;
        } else
            return false;//all motion model failed
    }

    //fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//already initialized in Frame constructor if this Track function is firstly called

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);//has CurrentFrame.mvpMapPoints[bestIdx2]=pMP; in this func. then it can use m-o BA

    // If few matches, uses a wider window search
    if(nmatches<20) {
      auto& curfmps_ref = mCurrentFrame.GetMapPointsRef();
      // it's important for SBP() will not rectify the alreay nice CurretFrame.mvpMapPoints
      fill(curfmps_ref.begin(), curfmps_ref.end(), static_cast<MapPoint*>(NULL));
      nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
    }

    if(nmatches<10)//20)//changed by JingWang
      return false;

    // Pose optimization. false: no need to compute marginalized for current Frame(motion-only), see VIORBSLAM paper (4)~(8)
    if(bMapUpdated){//we call this 2 frames'(FKF/FF) motion-only BA
      //not use Hessian matrix, it's ok
      Optimizer::PoseOptimization(&mCurrentFrame,mpLastKeyFrame,mpIMUInitiator->GetGravityVec(),false);//fixing lastKF(i), optimize curF(j)
    }else{
//       assert(mLastFrame.mbPrior==true||mLastFrame.mbPrior==false&&(mCurrentFrame.mnId==mnLastRelocFrameId+20||mnLastRelocFrameId==0));
      //unfix lastF(j): Hessian matrix exists, use prior Hessian to keep lastF(j)'s Pose stable, optimize j&j+1; fix lastF(j): optimize curF(j+1)
      Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame,mpIMUInitiator->GetGravityVec(),false);//last F unfixed/fixed when lastF.mOdomPreIntIMU.deltatij==0 or RecomputeIMUBiasAndCurrentNavstate(), save its Hessian
    }

    // Discard outliers
    int nmatchesMap = 0;
    const auto& curfmps = mCurrentFrame.GetMapPointMatches();
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(curfmps[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = curfmps[i];

                mCurrentFrame.EraseMapPointMatch(i);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                size_t n_cams = pMP->vbtrack_inview.size();
                pMP->vbtrack_inview.clear();
                pMP->vbtrack_inview.resize(n_cams, false);
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(curfmps[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)//we haven't designed tracking mode in VIO/VIEO, but we can just consider VIEO map as VEO then use VEO tracking mode! & VIO map as RGBD map then use RGBD tracking mode!
    {
        mbVO = nmatchesMap<6;//change to VO mode if the inlier MapPoint is too few i.e. robot goes to the new environment outside of the given map;old 10
        return nmatches>12;//Track ok when enough inlier matches;old 20
    }

    return nmatchesMap>=6;//10;//Track ok when enough inlier MapPoints, changed by JingWang
}
bool Tracking::PredictNavStateByIMU(bool bMapUpdated){
  assert(mpIMUInitiator->GetVINSInited());

  //Initialize NavState of mCurrentFrame
  // Map updated, optimize with last KeyFrame
  NavState &ns=mCurrentFrame.GetNavStateRef();
  if(bMapUpdated){
    // Get initial NavState&pose from Last KeyFrame
    ns=mpLastKeyFrame->GetNavState();
    PreIntegration(3);//preintegrate from LastKF to curF
//     cout<<"LastKF's pwb="<<ns.mpwb.transpose()<<endl;
  }
  // Map not updated, optimize with last Frame
  else{
    // Get initial pose from Last Frame
    ns=mLastFrame.GetNavStateRef();
    PreIntegration(1);//preintegrate from LastF to curF
//     cout<<"LastF's pwb="<<ns.mpwb.transpose()<<endl;
  }
  if (mCurrentFrame.GetIMUPreInt().mdeltatij==0){
    ns.mbg+=ns.mdbg;ns.mba+=ns.mdba;ns.mdbg=ns.mdba=Eigen::Vector3d::Zero();//here we just update bi to bi+dbi for next Frame will use fixedlastF mode
    //notice we can't keep this copy updation of mbi for too long!!!
    return false;//check PreIntegration() failed when mdeltatij==0, so mCurrentFrame.mTcw==cv::Mat()
  }

  using namespace Eigen;
  //const EncPreIntegrator &encpreint=mCurrentFrame.GetEncPreInt();
  //motion update/prediction by IMU motion model, see VIORBSLAM paper formula(3)
  Eigen::Vector3d gw=Converter::toVector3d(mpIMUInitiator->GetGravityVec());//gravity vector in world Frame/w/C0(not B0!!!)
  const IMUPreintegrator &imupreint=mCurrentFrame.GetIMUPreInt();//problem exits
  double deltat=imupreint.mdeltatij;

  Eigen::Matrix3d Rwb=ns.getRwb();//Rwbi
  ns.mpwb+=ns.mvwb*deltat+gw*(deltat*deltat/2)+Rwb*(imupreint.mpij+imupreint.mJgpij*ns.mdbg+imupreint.mJapij*ns.mdba);
  ns.mvwb+=gw*deltat+Rwb*(imupreint.mvij+imupreint.mJgvij*ns.mdbg+imupreint.mJavij*ns.mdba);
  // don't need to consider numerical error accumulation for it's not continuous multiplication or it will be normalized in BA
  Rwb*=imupreint.mRij*Sophus::SO3exd::exp(imupreint.mJgRij*ns.mdbg).matrix();//right update small increment: Rwbj=Rwbi*delta~Rij(bgi)=Rwbi*delta~Rij(bgi_bar)*Exp(JgRij*dbgi)
  ns.setRwb(Rwb);

  //Intialize bj of mCurrentFrame again used as bi_bar of next Frame & update pose matrices from NavState by Tbc
  ns.mbg+=ns.mdbg;//use bj_bar=bi_bar+dbi to set bj_bar(part of motion update, inspired by Random walk)
  ns.mba+=ns.mdba;//notice bi=bi_bar+dbi but we don't update bi_bar again to avoid preintegrating again
  ns.mdbg.setZero();ns.mdba.setZero();
  mCurrentFrame.UpdatePoseFromNS();//for VIE, we may use a complementary filter of encoder & IMU to predict NavState

//   cout<<"CurF's pwb="<<ns.mpwb.transpose()<<endl;
  return true;
}
bool Tracking::TrackLocalMapWithIMU(bool bMapUpdated){
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();

  SearchLocalPoints();

  // Optimize Pose
  if (mCurrentFrame.GetIMUPreInt().mdeltatij==0){
    cout<<redSTR"CurF.deltatij==0!In TrackLocalMapWithIMU(), Check!"<<whiteSTR<<endl;
    Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame);//motion-only BA
    mCurrentFrame.UpdateNavStatePVRFromTcw();//here is the imu data empty condition after imu's initialized, we must update NavState to keep continuous right Tbw after imu's initialized
  }else{
    // 2 frames' motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();
    if(bMapUpdated){
      Optimizer::PoseOptimization(&mCurrentFrame,mpLastKeyFrame,mpIMUInitiator->GetGravityVec(),true);//fixed last KF, save its Hessian
    }else{
//       assert(mLastFrame.mbPrior==true||mLastFrame.mbPrior==false&&(mCurrentFrame.mnId==mnLastRelocFrameId+20||mnLastRelocFrameId==0));
      Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame,mpIMUInitiator->GetGravityVec(),true);//last F unfixed/fixed when lastF.mOdomPreIntIMU.deltatij==0 or RecomputeIMUBiasAndCurrentNavstate()
      if (mLastFrame.GetIMUPreInt().mdeltatij==0) cout<<redSTR"LastF.deltatij==0!In TrackLocalMapWithIMU(), Check!"<<whiteSTR<<endl;
    }
  }
  //after IMU motion-only BA, we don't change bi to bi+dbi for reason that next Frame may(if imu data exists) still optimize dbi, so it's not necessary to update bi

  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for(int i=0; i<mCurrentFrame.N; i++)
  {
      if(curfmps[i])
      {
	  if(!mCurrentFrame.mvbOutlier[i])
	  {
	      curfmps[i]->IncreaseFound();
	      if(!mbOnlyTracking)
	      {
		  if(curfmps[i]->Observations()>0)
		      mnMatchesInliers++;
	      }
	      else
		  mnMatchesInliers++;
	  }
	  else if(mSensor==System::STEREO)//why not include System::RGBD?
            mCurrentFrame.EraseMapPointMatch(i);

      }
  }

  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently (recent 1s)
  if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && (mnMatchesInliers<10||mCurrentFrame.GetIMUPreInt().mdeltatij==0&&mnMatchesInliers<50))//50)
      return false;

  if(mnMatchesInliers<6)//30)//notice it's a class data member, changed by JingWang
      return false;
  else{
    double threInliers=15;//30;//TODO: check
    if (mCurrentFrame.GetIMUPreInt().mdeltatij==0&&mnMatchesInliers<threInliers)//if no imudata then it degenerates to TrackLocalMap()
      return false;
    else
      return true;
  }
}
void Tracking::RecomputeIMUBiasAndCurrentNavstate(){//see VIORBSLAM paper IV-E
  NavState& nscur=mCurrentFrame.GetNavStateRef();

  // Step1. Estimate gyr bias / see VIORBSLAM paper IV-A
  // compute initial IMU pre-integration for bgi_bar=0 as for the KFs' mOdomPreIntIMU in IMU Initialization
  size_t N=mv20pFramesReloc.size();
  listeig(IMUData)::const_iterator iterTmp=miterLastIMU;
  for(size_t i=0; i<N-1; ++i){
    unique_lock<mutex> lock(mMutexOdom);
    //so vKFInit[i].mOdomPreIntIMU is based on bg_bar=0,ba_bar=0; dbg=0 but dba/ba waits to be optimized
    PreIntegration<IMUData>(1,mlOdomIMU,miterLastIMU,mv20pFramesReloc[i],mv20pFramesReloc[i+1],nullptr);//actually we don't need to copy the data list!
  }
  Vector3d bgest;
  Optimizer::OptimizeInitialGyroBias<Frame>(mv20pFramesReloc, bgest);//though JingWang uses Identity() as Info
  // Update gyr bias of Frames
  assert(N==20);
  for(size_t i=0; i<N; ++i){
    mv20pFramesReloc[i]->GetNavStateRef().mbg=bgest;
    assert(mv20pFramesReloc[i]->GetNavStateRef().mdbg.norm()==0);
  }
  // Re-compute IMU pre-integration for bgi_bar changes to bgest from 0=>dbgi=0 see VIORBSLAM paper IV
  miterLastIMU=iterTmp;
  for(size_t i=0; i<N-1; ++i){
    unique_lock<mutex> lock(mMutexOdom);
    //so vKFInit[i].mOdomPreIntIMU is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba waits to be optimized
    PreIntegration<IMUData>(1,mlOdomIMU,miterLastIMU,mv20pFramesReloc[i],mv20pFramesReloc[i+1],nullptr);//actually we don't need to copy the data list!
  }
  if (!mlOdomEnc.empty()){//we update miterLastEnc to current Frame for the next Frame's Preintegration(1/3)!
    unique_lock<mutex> lock(mMutexOdom);
    listeig(EncData)::const_iterator iter=mlOdomEnc.end();
    iterijFind<EncData>(mlOdomEnc,mv20pFramesReloc[N-2]->mTimeStamp,iter,mdErrIMUImg);
    if (iter!=mlOdomEnc.end())
      miterLastEnc=iter;//update miterLastEnc pointing to the nearest(now,not next time) one of this frame / begin for next frame
    else
      miterLastEnc=--iter;
    PreIntegration<EncData>(1,mlOdomEnc,miterLastEnc,mv20pFramesReloc[N-2],mv20pFramesReloc[N-1],nullptr);
  }

  // Step 3. / See VIORBSLAM paper IV-C&E: Solve C*x=D for x=[ba] (3)x1 vector
  const cv::Mat Tcb=Converter::toCvMatInverse(Frame::mTbc);
  const cv::Mat gw=mpIMUInitiator->GetGravityVec();
  cv::Mat C=cv::Mat::zeros(3*(N-2),3,CV_32F);
  cv::Mat D=cv::Mat::zeros(3*(N-2),1,CV_32F);
  for(int i=0; i<N-2; i++){
    const Frame *pKF2=mv20pFramesReloc[i+1],*pKF3=mv20pFramesReloc[i+2];
    const IMUPreintegrator &imupreint12=pKF2->GetIMUPreInt(),&imupreint23=pKF3->GetIMUPreInt();
    //d means delta
    double dt12=imupreint12.mdeltatij;double dt23=imupreint23.mdeltatij;
    if (!dt12 || !dt23) continue;
    cv::Mat dp12=Converter::toCvMat(imupreint12.mpij);cv::Mat dp23=Converter::toCvMat(imupreint23.mpij);
    cv::Mat dv12=Converter::toCvMat(imupreint12.mvij);cv::Mat Jav12=Converter::toCvMat(imupreint12.mJavij);
    cv::Mat Jap12 = Converter::toCvMat(imupreint12.mJapij);cv::Mat Jap23=Converter::toCvMat(imupreint23.mJapij);
    cv::Mat Twb1=Converter::toCvMatInverse(mv20pFramesReloc[i]->GetTcwRef())*Tcb;//Twbi for pwbi&Rwbi, not necessary for clone()
    cv::Mat Twb2=Converter::toCvMatInverse(pKF2->GetcvTcwCst())*Tcb;cv::Mat Twb3=Converter::toCvMatInverse(pKF3->GetcvTcwCst())*Tcb;
    cv::Mat pb1=Twb1.rowRange(0,3).col(3);//pwbi=pwci_right_scaled+Rwc*tcb
    cv::Mat pb2=Twb2.rowRange(0,3).col(3);cv::Mat pb3=Twb3.rowRange(0,3).col(3);
    cv::Mat Rb1=Twb1.rowRange(0,3).colRange(0,3);//Rwbi
    cv::Mat Rb2=Twb2.rowRange(0,3).colRange(0,3);cv::Mat Rb3=Twb3.rowRange(0,3).colRange(0,3);
    // Stack to C/D matrix; zeta*ba=psi-(lambda*s+phi*dtheta)=psi2, Ci(3*3),Di/psi2(3*1)
    cv::Mat zeta=Rb2*Jap23*dt12+Rb1*Jav12*dt12*dt23-Rb1*Jap12*dt23;//3*3 notice here is Jav12, paper writes a wrong Jav23
    //note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
    cv::Mat psi2=(pb1-pb2)*dt23+(pb3-pb2)*dt12-Rb2*dp23*dt12-Rb1*dv12*dt12*dt23//notice here Rwci*pcb+(s=1)*pwci=pwbi
    +Rb1*dp12*dt23-(dt12*dt12*dt23+dt12*dt23*dt23)/2*(gw);//notice here use gw=Rwi*gI-Rwi*gI^
    zeta.copyTo(C.rowRange(3*i+0,3*i+3));
    psi2.copyTo(D.rowRange(3*i+0,3*i+3));
    //assert(dt12>0&&dt23>0);
  }
  // Use svd to compute C*x=D, x=[ba] 3x1 vector
  cv::Mat w2,u2,vt2;// Note w2 is 3x1 vector by SVDecomp()
  cv::SVD::compute(C,w2,u2,vt2,cv::SVD::MODIFY_A);
  cv::Mat w2inv=cv::Mat::eye(3,3,CV_32F);
  for(int i=0;i<3;++i){
    if(fabs(w2.at<float>(i))<1e-10){
      w2.at<float>(i) += 1e-10;
      cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
    }
    w2inv.at<float>(i,i)=1./w2.at<float>(i);
  }
  cv::Mat y=vt2.t()*w2inv*u2.t()*D;// Then y/x = vt'*winv*u'*D
  Vector3d bastareig=Converter::toVector3d(y);//here bai_bar=0, so dba=ba

  // Update acc bias, not necessary for this program!
  for(size_t i=0; i<N-1; i++){//for [N-1] is mCurrentFrame
    mv20pFramesReloc[i]->GetNavStateRef().mba=bastareig;
    assert(mv20pFramesReloc[i]->GetNavStateRef().mdba.norm()==0);
  }

  // Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
  // Compute Velocity of the mCurrentFrame(j/i+1) (but need (i)lastF's Velocity)
  Vector3d pwbjeig,vwbjeig;Matrix3d Rwbjeig;
  Frame* pCurF=mv20pFramesReloc[N-1];
  cv::Mat Twbj=Converter::toCvMatInverse(pCurF->GetcvTcwCst())*Tcb,Twbi=Converter::toCvMatInverse(mv20pFramesReloc[N-2]->GetcvTcwCst())*Tcb;
  const IMUPreintegrator& imupreint=pCurF->GetIMUPreInt();//the same condition as the paper
  double dt=imupreint.mdeltatij;                                		// deltatij
  cv::Mat pwbj=Twbj.rowRange(0,3).col(3);pwbjeig=Converter::toVector3d(pwbj);	// pwbj
  Rwbjeig=Converter::toMatrix3d(Twbj.rowRange(0,3).colRange(0,3));
  cv::Mat pwbi=Twbi.rowRange(0,3).col(3),Rwbi=Twbi.rowRange(0,3).colRange(0,3);	//pwbi,Rwbi
  cv::Mat vwbi=-1./dt*((pwbi-pwbj)+dt*dt/2*gw+Rwbi*Converter::toCvMat(Vector3d(imupreint.mpij+imupreint.mJapij*bastareig)));//-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(deltap~ij+Japij*dbai))
  // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
  vwbjeig=Converter::toVector3d(vwbi+gw*dt+Rwbi*Converter::toCvMat(Vector3d(imupreint.mvij+imupreint.mJavij*bastareig)));//vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)

  assert(mv20pFramesReloc[N-1]->mnId==mCurrentFrame.mnId);

  // Set NavState of Current Frame, P/R/V/bg/ba/dbg/dba
  nscur.mpwb=pwbjeig;nscur.setRwb(Rwbjeig);
  nscur.mvwb=vwbjeig;
  nscur.mbg=bgest;nscur.mba=bastareig;//let next PreIntegration() has a fine bi_bar
  nscur.mdbg=nscur.mdba=Vector3d::Zero();
}

//created by zzh over.

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
    mbRelocBiasPrepare(false),mnLastOdomKFId(0),mbKeyFrameCreated(false)//zzh
{
  // Load camera parameters from settings file
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  // load Tbc,Tbo refer the Jing Wang's configparam.cpp
  cv::FileNode fnT[2] = {fSettings["Camera.Tbc"], fSettings["Camera.Tce"]};
  Eigen::Matrix3d eigRtmp;
  mTbc = cv::Mat::eye(4, 4, CV_32F);
  for (int i = 0; i < 2; ++i) {
    if (fnT[i].empty()) {
      PRINT_INFO_MUTEX( redSTR "No Tbc/Tce, please check if u wanna use VIO!" << whiteSTR << endl);
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
  PRINT_INFO_MUTEX( "Tbc:" << mTbc << endl);
  Frame::mTbc = mTbc.clone();
  cv::Mat Tcb = Converter::toCvMatInverse(mTbc);
  Frame::meigRcb = Sophus::SO3exd(Converter::toMatrix3d(Tcb.rowRange(0, 3).colRange(0, 3))).matrix();
  Frame::meigtcb = Converter::toVector3d(Tcb.rowRange(0, 3).col(3));
  // load Sigma etad & etawi & gd,ad,bgd,bad & if accelerate needs to *9.81
  cv::FileNode fnSig[3] = {fSettings["IMU.SigmaI"], fSettings["IMU.sigma"], fSettings["IMU.dMultiplyG"]};
  if (fnSig[0].empty() || fnSig[1].empty() || fnSig[2].empty())
    PRINT_INFO_MUTEX( redSTR "No IMU.sigma or IMU.dMultiplyG!" << whiteSTR << endl);
  else {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) eigRtmp(i, j) = fnSig[0][i * 3 + j];
    double sigma2tmp[4] = {fnSig[1][0], fnSig[1][1], fnSig[1][2], fnSig[1][3]};
    for (int i = 0; i < 4; ++i) sigma2tmp[i] *= sigma2tmp[i];
    IMUDataDerived::SetParam(eigRtmp, sigma2tmp, fnSig[2],
                             fSettings["IMU.dt_cov_noise_fix"].empty() ? 0 : fSettings["IMU.dt_cov_noise_fix"],
                             fSettings["IMU.freq_hz"].empty() ? 0 : fSettings["IMU.freq_hz"]);
  }
  // load rc,vscale,Sigma etad
  cv::FileNode fnEnc[4] = {fSettings["Encoder.scale"], fSettings["Encoder.rc"], fSettings["Encoder.sigma"]};
  if (fnEnc[0].empty() || fnEnc[1].empty() || fnEnc[2].empty())
    PRINT_INFO_MUTEX( redSTR "No Encoder.simga or Encoder.scale or Encoder.rc!" << whiteSTR << endl);
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
    PRINT_INFO_MUTEX( redSTR "No Camera.delaytoimu & delaytoenc & delayForPolling!" << whiteSTR << endl);
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
    PRINT_INFO_MUTEX( redSTR "No ErrIMUImg!" << whiteSTR << endl);
    mdErrIMUImg = 1. / fps;
  } else {
    mdErrIMUImg = (double)fnErrIMUImg;
  }

  // created by zzh over.

  cv::FileNode nodeCameraName = fSettings["Camera.type"];
  string sCameraName = "Pinhole";
  if (!nodeCameraName.empty()) sCameraName = string(nodeCameraName);
  PRINT_INFO_MUTEX( "Camera.type=" << sCameraName << endl);
  mpCameras.resize(4);  // TODO: detect the size
  mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  for (int i = 0; i < 4; ++i) {
    bool bexist_cam = false;
    if (sCameraName == "KannalaBrandt8") {
      pSys->usedistort_ = true;  // TODO: now fisheye only usedistort_ is confirmed
      bexist_cam = KannalaBrandt8::ParseCamParamFile(fSettings, i, mpCameras[i], !i ? &mK : nullptr, nullptr);
    } else {
      if (sCameraName == "Pinhole") {
        CV_Assert(!pSys->usedistort_);
        cv::Mat* pDistCoef = !i ? &mDistCoef : nullptr;
        bexist_cam = Pinhole::ParseCamParamFile(fSettings, i, mpCameras[i], !i ? &mK : nullptr, pDistCoef);
      } else if (sCameraName == "Radtan") {
        cv::Mat* pDistCoef = (!pSys->usedistort_ && !i) ? &mDistCoef : nullptr;
        bexist_cam = Radtan::ParseCamParamFile(fSettings, i, mpCameras[i], !i ? &mK : nullptr, pDistCoef);
        // pSys->usedistort_ = false;
      } else
        CV_Assert(0 && "Unsupported Camera Model");
    }
    if (!bexist_cam) {
      mpCameras.resize(i);
      break;
    }
  }
  if (mpCameras.size() > 1) mpFrameDrawer->showallimages_ = true;
  PRINT_INFO_MUTEX( "Cam size = " << mpCameras.size() << endl);
  CLEAR_DEBUG_INFO("start debug:", imu_tightly_debug_path, "debug.txt");

  mbf = fSettings["Camera.bf"];

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  PRINT_INFO_MUTEX( "- fps: " << fps << endl);

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if (mbRGB)
    PRINT_INFO_MUTEX( "- color order: RGB (ignored if grayscale)" << endl);
  else
    PRINT_INFO_MUTEX( "- color order: BGR (ignored if grayscale)" << endl);

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
    mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  PRINT_INFO_MUTEX( endl << "ORB Extractor Parameters: " << endl);
  PRINT_INFO_MUTEX( "- Number of Features: " << nFeatures << endl);
  PRINT_INFO_MUTEX( "- Scale Levels: " << nLevels << endl);
  PRINT_INFO_MUTEX( "- Scale Factor: " << fScaleFactor << endl);
  PRINT_INFO_MUTEX( "- Initial Fast Threshold: " << fIniThFAST << endl);
  PRINT_INFO_MUTEX( "- Minimum Fast Threshold: " << fMinThFAST << endl);

  if (sensor == System::STEREO || sensor == System::RGBD) {
    mThDepth = mbf * (float)fSettings["ThDepth"] / mK.at<float>(0, 0);
    PRINT_INFO_MUTEX( endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl);
  }

  if (sensor == System::RGBD) {
    mDepthMapFactor = fSettings["DepthMapFactor"];
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const vector<cv::Mat> &ims, const double &timestamp, const bool inputRect) {
  mtmGrabDelay = chrono::steady_clock::now();  // zzh
  int n_cams = ims.size();
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
    }
  }

  mCurrentFrame = Frame(mImGrays, timestamp, mpORBextractors, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
                        &preint_imu_kf_, &preint_enc_kf_, inputRect ? nullptr : &mpCameras, System::usedistort_);

  Track();

  return mCurrentFrame.GetTcwRef().clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp) {
  mtmGrabDelay = chrono::steady_clock::now();  // zzh
  mImGrays[0] = imRGB;
  cv::Mat imDepth = imD;

  // may be improved here!!!
  if (mImGrays[0].channels() == 3) {
    if (mbRGB)
      cvtColor(mImGrays[0], mImGrays[0], CV_RGB2GRAY);
    else {
      cvtColor(mImGrays[0], mImGrays[0], CV_BGR2GRAY);
    }
  } else if (mImGrays[0].channels() == 4) {
    if (mbRGB)
      cvtColor(mImGrays[0], mImGrays[0], CV_RGBA2GRAY);
    else
      cvtColor(mImGrays[0], mImGrays[0], CV_BGRA2GRAY);
  }

  if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  mCurrentFrame = Frame(mImGrays[0], imDepth, timestamp, mpORBextractors[0], mpORBVocabulary, mK, mDistCoef, mbf,
                        mThDepth, &preint_imu_kf_, &preint_enc_kf_);  // here extracting the ORB features of ImGray

  cv::Mat img[2] = {imRGB.clone(), imD.clone()};
  Track(img);

  return mCurrentFrame.GetTcwRef().clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
  mtmGrabDelay = chrono::steady_clock::now();  // zzh
  mImGrays[0] = im;

  if (mImGrays[0].channels() == 3) {
    if (mbRGB)
      cvtColor(mImGrays[0], mImGrays[0], CV_RGB2GRAY);
    else
      cvtColor(mImGrays[0], mImGrays[0], CV_BGR2GRAY);
  } else if (mImGrays[0].channels() == 4) {
    if (mbRGB)
      cvtColor(mImGrays[0], mImGrays[0], CV_RGBA2GRAY);
    else
      cvtColor(mImGrays[0], mImGrays[0], CV_BGRA2GRAY);
  }

  if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
    mCurrentFrame = Frame(mImGrays[0], timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
                          &preint_imu_kf_, &preint_enc_kf_);
  else
    mCurrentFrame = Frame(mImGrays[0], timestamp, mpORBextractors[0], mpORBVocabulary, mK, mDistCoef, mbf, mThDepth,
                          &preint_imu_kf_, &preint_enc_kf_);

  Track();

  return mCurrentFrame.GetTcwRef().clone();
}

void Tracking::Track(cv::Mat img[2])//changed a lot by zzh inspired by JingWang
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    //Get Map Mutex, we don't hope part of Map to be changed when we write(but some operations like add is
    // ok) and read it to get new KFs/MPs
    //  core reason is for local map tracking and mono initialize will call GBA with nloopKF=0
    //  and for AddNewKF should have the same map change id with Preintegration lastF/KF's ns.bgba
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    //delay control
    {
        char sensorType=0;//0 for nothing, 1 for encoder, 2 for IMU, 3 for encoder+IMU
        if (mpIMUInitiator->GetSensorEnc()) ++sensorType;
        if (mpIMUInitiator->GetSensorIMU()) sensorType+=2;
        unique_lock<mutex> lock2(mMutexOdom);
        if (sensorType==1&&(mlOdomEnc.empty()||mlOdomEnc.back().mtm<mCurrentFrame.mTimeStamp)||
        sensorType==2&&(mlOdomIMU.empty()||mlOdomIMU.back().mtm<mCurrentFrame.mTimeStamp)||
        sensorType==3&&(mlOdomEnc.empty()||mlOdomEnc.back().mtm<mCurrentFrame.mTimeStamp||mlOdomIMU.empty()||mlOdomIMU.back().mtm<mCurrentFrame.mTimeStamp)){
        lock2.unlock();//then delay some ms to ensure some Odom data to come if it runs well
        mtmGrabDelay+=chrono::duration_cast<chrono::nanoseconds>(chrono::duration<double>(mDelayCache));//delay default=20ms
        while (chrono::steady_clock::now()<mtmGrabDelay) usleep(1000);//allow 1ms delay error
        }//if still no Odom data comes, deltax~ij will be set unknown
    }

    // Different operation, according to whether the map is updated
    bool bMapUpdated = false;
    static int premapid = 0;
    int curmapid = mpMap->GetLastChangeIdx();
    if (curmapid != premapid) {
        premapid = curmapid;
        bMapUpdated = true;
    }

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization(img);
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK||mState==ODOMOK)//||mState==MAP_REUSE_RELOC)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();//so use the replaced ones

                if (mpIMUInitiator->GetVINSInited()){//if IMU info(bgi,gw,bai) is intialized use IMU motion model
                    if (!mbRelocBiasPrepare){//but still need >=20 Frames after reloc, bi becomes ok for IMU motion update(calculated in 19th Frame after reloc. KF)
                        bOK=TrackWithIMU(bMapUpdated);
                        if(!bOK){
                            cout<<redSTR"TrackWitIMU() failed!"<<whiteSTR<<endl;
                            if (bOK=TrackReferenceKeyFrame()) mCurrentFrame.UpdateNavStatePVRFromTcw();//don't forget to update NavState though bg(bg_bar,dbg)/ba(ba_bar,dba) cannot be updated; 6,7
                        }
                    }else{//<20 Frames after reloc then use pure-vision tracking, but notice we can still use Encoder motion update!
                        //we don't preintegrate Enc or IMU here for a simple process with mdeltatij==0! when bOK==false
                        if (mVelocity.empty()){
                            if (bOK=TrackReferenceKeyFrame()) mCurrentFrame.UpdateNavStatePVRFromTcw();//match with rKF, use lF as initial & m-o BA
                            cout<<fixed<<setprecision(6)<<redSTR"TrackRefKF()"whiteSTR<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
                        }else{
                            bOK=TrackWithMotionModel();//match with lF, use v*lF(velocityMM) as initial & m-o BA
                            if(!bOK){
                                bOK=TrackReferenceKeyFrame();
                                cout<<redSTR"TrackRefKF()2"whiteSTR<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
                            }
                            if (bOK) mCurrentFrame.UpdateNavStatePVRFromTcw();
                        }
                    }
                }else{
                    if (!mLastFrame.GetTcwRef().empty()) GetVelocityByEnc(bMapUpdated);//try to utilize the Encoder's data
                    else cout<<redSTR<<"LastFrame has no Tcw!"<<whiteSTR<<endl;
                    if(mVelocity.empty()){// || mCurrentFrame.mnId<mnLastRelocFrameId+2){//if last frame relocalized, there's no motion could be calculated, so I think 2nd condition is useless
//                        if (!mVelocity.empty()) cerr<<redSTR"Error in Velocity.empty()!!!"<<endl;//check if right
                        bOK = TrackReferenceKeyFrame();//match with rKF, use lF as initial & m-o BA
                        cout<<fixed<<setprecision(6)<<redSTR"TrackRefKF()"whiteSTR<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
                    }else{
                        bOK = TrackWithMotionModel();//match with lF, use v*lF(velocityMM) as initial & m-o BA
                        if(!bOK){
                            bOK = TrackReferenceKeyFrame();
                            cout<<redSTR"TrackRefKF()2"whiteSTR<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
                        }
//                        cout<<mCurrentFrame.mnId<<endl;
                    }
                }
            }
            else//mState==LOST or MAP_REUSE
            {
                if (mState==MAP_REUSE) PreIntegration();//clear cached Odom List
                bOK = Relocalization();
                cout<<greenSTR"Relocalization()"whiteSTR<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
            if (mState==MAP_REUSE){
                PreIntegration();
                bOK=Relocalization();
                cout<<greenSTR"Relocalization()"whiteSTR<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
            }else if(mState==LOST)
            {
                bOK = Relocalization();
                cout<<"Lost--Local. Mode"<<endl;
            }
            else if (mpIMUInitiator->GetVINSInited()){//if IMU info(bgi,gw,bai) is intialized use IMU motion model
                // Todo: Add VIO & VIEO Tracking Mode, it's not key for our target, so we haven't finished this part
                cout<<redSTR<<"Entering Wrong Tracking Mode With VIO/VIEO, Please Check!"<<endl;
                assert(0);
            }else{
                if (!mLastFrame.GetTcwRef().empty()) GetVelocityByEnc(bMapUpdated);//try to utilize the Encoder's data
                else cout<<redSTR<<"LastFrame has no Tcw!"<<whiteSTR<<endl;
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.GetMapPointMatches();
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetTcwRef().clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.GetMapPointsRef() = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                          const auto& curfmps = mCurrentFrame.GetMapPointMatches();
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(curfmps[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    curfmps[i]->IncreaseFound();//++mnFound in this map point
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                        cout<<"Reloc--Local. Mode"<<endl;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;//firstly use last mState==OK Frame.mpReferenceKF, maybe use most covisible KF as the (mCurrentFrame.)mpReferenceKF in TrackLocalMap()

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking) {
          if (bOK) {
            if (!mpIMUInitiator->GetVINSInited() ||
                mbRelocBiasPrepare)  // if imu not intialized(including relocalized bias recomputation)
              bOK = TrackLocalMap();
            else
              bOK = TrackLocalMapWithIMU(bMapUpdated);

            if (!bOK) {
              cout << redSTR "TrackLocalMap() failed!" whiteSTR << endl;
            }
          }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK){
//            if (mpIMUInitiator->GetVINSInited()&&mState==MAP_REUSE||mState==MAP_REUSE_RELOC) mState=MAP_REUSE_RELOC;
//            else
            mState=OK;
            //Add Frames to re-compute IMU bias after reloc
            if(mbRelocBiasPrepare){
                cout<<redSTR<<" Relocalization Recomputation Preparing..."<<whiteSTR<<endl;
                mv20pFramesReloc.push_back(new Frame(mCurrentFrame));

                // Before creating new keyframe
                // Use 20 consecutive frames to re-compute IMU bias, see VIORBSLAM paper IV-E
                if(mCurrentFrame.mnId == mnLastRelocFrameId+20-1){
                    RecomputeIMUBiasAndCurrentNavstate();// Update NavState of CurrentFrame for it's only used in Tracking thread
                    // Clear flag and Frame buffer
                    mbRelocBiasPrepare=false;
                    for (int i=0;i<mv20pFramesReloc.size();++i) delete mv20pFramesReloc[i];
                    mv20pFramesReloc.clear();
//                    if (mState==MAP_REUSE_RELOC) mState=OK;
                }
            }
        }else{//we shouldn't make it LOST for robustness
            //mState=LOST;
            //use Odom data to get mCurrentFrame.mTcw
            if (mCurrentFrame.GetEncPreInt().mdeltatij>0){//though it may introduce error, it ensure the completeness of the Map
                TrackWithOnlyOdom(bMapUpdated);
            }else{
              // for when enc ok, always OK/ODOMOK, no mbRelocBiasPrepare entered for no relocalization is called or
              // always Preintegration(1/3) before, but when enc failed we need to preint kf2kf next relocalization time
              for (auto& brecompute : brecompute_kf2kfpreint_) brecompute = true;
                // Clear Frame vectors for reloc bias computation
                if(mv20pFramesReloc.size()>0){
                    for (int i=0;i<mv20pFramesReloc.size();++i) delete mv20pFramesReloc[i];
                    mv20pFramesReloc.clear();
                }
//                if (mState==MAP_REUSE_RELOC) mState=MAP_REUSE;
                if (mState==MAP_REUSE) return;//if MAP_REUSE, we keep mState until it's relocalized
                else mState=LOST;//if LOST, the system can only be recovered through relocalization module, so no need to set mbRelocBiasPrepare
            }
        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.GetTcwRef().empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.GetTcwRef()*LastTwc;//Tc2c1/Tcl
            }
            else{
                mVelocity = cv::Mat();//can use odometry data here!
                //cout<<redSTR"Error in mVelocity=cv::Mat()"<<whiteSTR<<endl;
            }

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetTcwRef());

          // Clean VO matches, related to Localization mode
          const auto& curfmps = mCurrentFrame.GetMapPointMatches();
          for(int i=0; i<mCurrentFrame.N; i++) {
            MapPoint* pMP = curfmps[i];
            if (pMP)                        // if the mappoint of th i feature is added in to the pMap
              if (pMP->Observations() < 1)  // if now no KF observes it, delete it from the vmap in Frame
              {
                mCurrentFrame.mvbOutlier[i] = false;  // though motion-only BA will initialize this, here is for new MapPoints created by CreateNewKeyFrame(), they must be inliers
                mCurrentFrame.EraseMapPointMatch(i);
              }
          }

            // Delete temporal MapPoints, for they're only used for current Tcw calculation, related to Localization mode
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe!!
            if(NeedNewKeyFrame()){
                CreateNewKeyFrame(img);//only create the only CurrentFrame viewed MapPoints without inliers+outliers in mpMap, to avoid possibly replicated MapPoints
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
          const auto &curfmps2 = mCurrentFrame.GetMapPointMatches();
          for(int i=0; i<mCurrentFrame.N;i++) {        // new created MPs' mvbOutlier[j] is default false
            if (curfmps2[i] && mCurrentFrame.mvbOutlier[i])  // delete the final still outliers mappoints in Frame
              mCurrentFrame.EraseMapPointMatch(i);
          }

            //For the 1st frame's tracking strategy after IMU initialized(notice no prior IMU Hessian matrix), JingWang chooses tracking with lastKF, \
            If the transition frame's tracking is unstable, we can improve it through our no Hessian matrix strategy~
        }else if (mState==ODOMOK) {  // if it's lost in Camera mode we use Odom mode
          // not necessary to update motion model for mVelocity is already got through odom data

          mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetTcwRef());

          // Clean VO matches, related to Localization mode
          const auto& curfmps = mCurrentFrame.GetMapPointMatches();
          for (int i = 0; i < mCurrentFrame.N; i++) {
            MapPoint* pMP = curfmps[i];
            if (pMP)                        // if the mappoint of th i feature is added in to the pMap
              if (pMP->Observations() < 1)  // if now no KF observes it, delete it from the vmap in Frame
              {
                mCurrentFrame.mvbOutlier[i] = false;  // though motion-only BA will initialize this, here is for new MapPoints created by CreateNewKeyFrame(), they must be inliers
                mCurrentFrame.EraseMapPointMatch(i);
              }
          }

          if (NeedNewKeyFrame()) {
            CreateNewKeyFrame(img);  // only create the only CurrentFrame viewed MapPoints without inliers+outliers in mpMap, to avoid possibly replicated MapPoints
          }
          const auto& curfmps2 = mCurrentFrame.GetMapPointMatches();
          for (int i = 0; i < mCurrentFrame.N; i++) {        // new created MPs' mvbOutlier[j] is default false
            if (curfmps2[i] && mCurrentFrame.mvbOutlier[i])  // delete the final still outliers mappoints in Frame
              mCurrentFrame.EraseMapPointMatch(i);
          }
        }

        // Reset if the camera get lost soon after initialization
        if(!mbOnlyTracking&&mState==LOST)
        {
            bool autoreset=mpIMUInitiator->GetSensorIMU()&&!mpIMUInitiator->mbUsePureVision?!mpIMUInitiator->GetVINSInited():mpMap->KeyFramesInMap()<=5;
            if(autoreset)
            {
                PRINT_INFO_MUTEX( redSTR"Track lost soon after initialisation, reseting..." <<whiteSTR<< endl);
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)//when mCurrentFrame is not KF but it cannot see common MPs in local MPs(e.g. it has all new MPs but it's not inserted as new KF & you can get its mTcw through other odometry)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);//copy constructor for some Mat.clone() and vec<>[][] deep copy, notice mLastFrame is also set in XXXInitialization()!
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.GetTcwRef().empty())
    {
        cv::Mat Tcr = mCurrentFrame.GetTcwRef()*mCurrentFrame.mpReferenceKF->GetPoseInverse();//when it's lost but get an initial pose through motion-only BA , it can still recover one low-quality estimation, used in UpdateLastFrame()
        mlRelativeFramePoses.push_back(Tcr);
        const NavStated &ns = mCurrentFrame.GetNavStateRef();
        relative_frame_bvwbs_.push_back(ns.mRwb.inverse() * ns.mvwb);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);//false if it isn't lost, when it has Tcw, it stll can be LOST for not enough inlier MapPoints
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());//I think actually it's unused
        relative_frame_bvwbs_.push_back(relative_frame_bvwbs_.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);//it's key value to judge
    }
}


void Tracking::StereoInitialization(cv::Mat img[2])
{
    PreIntegration();//PreIntegration Intialize, zzh

    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
	if (img){ pKFini->Img[0]=img[0];pKFini->Img[1]=img[1];}

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
	//every points with depth info!
        if (mCurrentFrame.mv3Dpoints.empty()) {
          for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
              cv::Mat x3D = mCurrentFrame.UnprojectStereo(
                  i);  // use camera intrinsics to backproject the piont from (u,v) to x3Dw
              MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
              pNewMP->AddObservation(pKFini,
                                     i);  // add which KF with the order of features in it has observed this mappoint
              pKFini->AddMapPoint(pNewMP, i);
              pNewMP->ComputeDistinctiveDescriptors();  // choose the observed descriptor having the least median distance to the left
              pNewMP->UpdateNormalAndDepth();           // calc the mean viewing direction and max/min dist?
              mpMap->AddMapPoint(pNewMP);

              mCurrentFrame.AddMapPoint(pNewMP, i); // the realization of the AddMapPiont in class Frame
            }
          }
        } else {
          // TODO: only left this
          for (int k = 0; k < mCurrentFrame.mv3Dpoints.size(); ++k) {
            if (mCurrentFrame.goodmatches_[k]) {
              int i = mCurrentFrame.mapidxs2n_[k];
              CV_Assert(-1 != i);
              cv::Mat x3D = mCurrentFrame.UnprojectStereo(
                  i);  // use camera intrinsics to backproject the piont from (u,v) to x3Dw
              MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpMap);
              auto idxs = mCurrentFrame.mvidxsMatches[k];
              bool icheck = false;
              PRINT_DEBUG_INFO_MUTEX("addmap-1, kf0:", imu_tightly_debug_path, "debug.txt");
              CV_Assert(idxs.size());
              for (int cami=0;cami< idxs.size(); ++cami) {
                if (-1 != idxs[cami]) {
                  auto icami = mCurrentFrame.mapin2n_[cami][idxs[cami]];
                  if (icami == i) icheck = true;
                  // add which KF with the order of features in it has observed this mappoint
                  pNewMP->AddObservation(pKFini, icami);
                  pKFini->AddMapPoint(pNewMP, icami);
                  mCurrentFrame.AddMapPoint(pNewMP, icami); // the realization of the AddMapPiont in class Frame
                }
              }
              PRINT_DEBUG_INFO_MUTEX(endl, imu_tightly_debug_path, "debug.txt");
              CV_Assert(icheck);
              pNewMP->ComputeDistinctiveDescriptors();  // choose the observed descriptor having the least median distance to the left
              pNewMP->UpdateNormalAndDepth();           // calc the mean viewing direction and max/min dist?
              mpMap->AddMapPoint(pNewMP);
            }
          }
        }

        PRINT_INFO_MUTEX( "New map created with " << mpMap->MapPointsInMap() << " points" << endl);

        mpLocalMapper->InsertKeyFrame(pKFini);//add it to the local mapper KF list

        mCurrentFrame.mpReferenceKF = pKFini;//I think it should be put before mLastFrame=!
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

//         mvpLocalKeyFrames.push_back(pKFini);//unused here
        mvpLocalMapPoints=mpMap->GetAllMapPoints();//for MapDrawer.cc
        mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetTcwRef());

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
	    PreIntegration();//PreIntegration Intialize, Clear IMUData buffer, zzh

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

//             if(mpInitializer) delete mpInitializer;//useless

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB,pKFini);//zzh, it's very important! or assert failed in KF->SetBadFlag()

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.AddMapPoint(pMP, mvIniMatches[i]);
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    PRINT_INFO_MUTEX( "New Map created with " << mpMap->MapPointsInMap() << " points" << endl);

    Optimizer::GlobalBundleAdjustment(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        PRINT_INFO_MUTEX( "Wrong initialization, reseting..." << endl);
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    //mpLastKeyFrame = pKFcur;//zzh, used later

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;//next 2KFs' ComputePreInt()/Preintegration(2) need this variable set to current KF
    mCurrentFrame.mpReferenceKF = pKFcur;

    mpLastKeyFrame=pKFini;//zzh, initial KF doesn't need to ComputePreInt(), and PreIntegration()/Clear IMUData buffer before initial KF is done in MonocularInitialization()
    PreIntegration(2);//zzh, this will automatically Clear IMUData buffer before mCurrentFrame/pKFcur, must be put after Wrong return
    mpLastKeyFrame=pKFcur;//zzh, replaced here

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame() {
  // Set of MapPoints already found in the KeyFrame
  const auto& lfmps = mLastFrame.GetMapPointMatches();
#ifndef CHECK_REPLACE_ALL
  set<pair<MapPoint*, size_t>> spAlreadyFound = mLastFrame.GetMapPointsCami();
#endif
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPoint* pMP = lfmps[i];

    if (pMP) {
      MapPoint* pRep = pMP->GetReplaced();
      if (pRep) {
        size_t cami = mLastFrame.mapin2n_.size() <= i ? 0 : get<0>(mLastFrame.mapn2in_[i]);
        CV_Assert(pMP->isBad());
        while (pRep && pRep->isBad()) {
#ifndef CHECK_REPLACE_ALL
          // solve0: lastframe seen mp-1->mp0->mp1, mp0->mp1, (mp0 seen in lastframe), then mp-1 should be directly erased
          if (spAlreadyFound.end() != spAlreadyFound.find(make_pair(pRep, cami))) break;
#endif
          pRep = pRep->GetReplaced();
        }
        // notice that 2features in the same frame could see the same mp, caused by replace op.
        bool bcontinue = false;
        if (!pRep  // solve1: lastframe seen mp-1->mp0->mp1->null, (mp1 bad), then mp-1 should be directly erased
#ifndef CHECK_REPLACE_ALL
            || spAlreadyFound.end() != spAlreadyFound.find(make_pair(pRep, cami))  // solve0
#endif
        ) {
          bcontinue = true;
        }
#ifndef CHECK_REPLACE_ALL
        else if (mnLastKeyFrameId == mLastFrame.mnId) {
          // solve2: lba curkf/vpMapPoints1 4961->607 in pKF(18) then 4959->607 in pKF(19)
          if (!pRep->IsInKeyFrame(mpLastKeyFrame, i, cami)) bcontinue = true;
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
        PRINT_DEBUG_INFO_MUTEX(i << "lfreplace" << pRep->mnId << ":bad" << (int)pRep->isBad(), imu_tightly_debug_path,
                               "debug.txt");
#ifndef CHECK_REPLACE_ALL
        if (spAlreadyFound.end() != spAlreadyFound.find(make_pair(pRep, cami)))
          PRINT_DEBUG_INFO_MUTEX(endl, imu_tightly_debug_path, "debug.txt");
          //        CV_Assert(spAlreadyFound.end() == spAlreadyFound.find(pRep));
#endif
        mLastFrame.ReplaceMapPointMatch(i, pRep);  // if it's replaced by localMapper, use the replaced one
                                                   //        spAlreadyFound.insert(pRep);
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame(int thInMPs,int thMatch)
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);//stricter then SBP in TrackLocalMap()
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);//match with rKF,rectify the vpMapPointMatches, SBBoW better than SBP for the mCurrentFrame.Tcw is unknown
  PRINT_DEBUG_INFO_MUTEX("math num="<<nmatches<<endl, imu_tightly_debug_path, "debug.txt");

    if(nmatches<thMatch)//15)//looser than 20 in TrackWithMotionModel()
        return false;

    mCurrentFrame.GetMapPointsRef() = vpMapPointMatches;//use temporary vector<MapPoint*> for not believe SBBow() so much
    mCurrentFrame.SetPose(mLastFrame.GetTcwRef());//but use lF as the initial value for BA

  PRINT_DEBUG_INFO_MUTEX("bef opt="<<nmatches<<endl, imu_tightly_debug_path, "debug.txt");
    int num_inliers = Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA
  PRINT_DEBUG_INFO_MUTEX( "inliers="<<num_inliers<<endl, imu_tightly_debug_path, "debug.txt");

    // Discard outliers
    int nmatchesMap = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(curfmps[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = curfmps[i];//use temporary pointer to avoid mutex problem?

                mCurrentFrame.EraseMapPointMatch(i);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                size_t n_cams = pMP->vbtrack_inview.size();
                pMP->vbtrack_inview.clear();
                pMP->vbtrack_inview.resize(n_cams, false);
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;//useless here
            }
            else if(curfmps[i]->Observations()>0)//where EraseObservation()/SetBadFlag()?
                nmatchesMap++;
        }
    }

    return nmatchesMap>=thInMPs;//10;//Track ok when enough inlier MapPoints
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    //similar with the part in CreateNewKeyFrame()
    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    const auto &lfmps = mLastFrame.GetMapPointMatches();
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = lfmps[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew) {
          cv::Mat x3D = mLastFrame.UnprojectStereo(i);
          MapPoint* pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);  // different here, TODO: unify it

          size_t ididxs = mCurrentFrame.GetMapn2idxs(i);
          if (-1 == ididxs) {
            mLastFrame.AddMapPoint(pNewMP, i);
          } else {
            auto idxs = mCurrentFrame.mvidxsMatches[ididxs];
            bool icheck = false;
            for (int cami = 0; cami < idxs.size(); ++cami) {
              if (-1 != idxs[cami]) {
                auto icami = mCurrentFrame.mapin2n_[cami][idxs[cami]];
                if (icami == i) icheck = true;
                mLastFrame.AddMapPoint(pNewMP, icami);
              }
            }
            CV_Assert(icheck);
          }
          //pNewMP->ComputeDistinctiveDescriptors(); // TODO: compute and update for fisheye
          //pNewMP->UpdateNormalAndDepth();

          mlpTemporalPoints.push_back(pNewMP);
          nPoints++;
        }
        else
        {
            nPoints++;//can be optimzed here
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);//here 0.9 is useless

    // Update last frame pose according to its reference keyframe for rKF may be rectified
    // Create "visual odometry" points if in Localization Mode

    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.GetTcwRef());//Tc2c1*Tc1w

  //fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//already initialized in Frame constructor if this Track function is firstly called

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);//has CurrentFrame.mvpMapPoints[bestIdx2]=pMP; in this func. then it can use m-o BA
  PRINT_DEBUG_INFO_MUTEX( "math num2="<<nmatches<<endl, imu_tightly_debug_path, "debug.txt");

  // If few matches, uses a wider window search
  if(nmatches<20) {
    auto& curfmps_ref = mCurrentFrame.GetMapPointsRef();
    // it's important for SBP() will not rectify the alreay nice CurretFrame.mvpMapPoints
    fill(curfmps_ref.begin(), curfmps_ref.end(), static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
  }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    int num_inliers = Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame);//motion-only BA
//     Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA
    PRINT_DEBUG_INFO_MUTEX( "inliers2="<< num_inliers<<endl, imu_tightly_debug_path, "debug.txt");

    // Discard outliers
    int nmatchesMap = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(curfmps[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = curfmps[i];

                mCurrentFrame.EraseMapPointMatch(i);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                size_t n_cams = pMP->vbtrack_inview.size();
                pMP->vbtrack_inview.clear();
                pMP->vbtrack_inview.resize(n_cams, false);
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(curfmps[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;//change to VO mode if the inlier MapPoint is too few i.e. robot goes to the new environment outside of the given map
        return nmatches>20;//Track ok when enough inlier matches
    }

    return nmatchesMap>=10;//Track ok when enough inlier MapPoints
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    PRINT_DEBUG_INFO_MUTEX("SLP" << endl, imu_tightly_debug_path, "debug.txt");
    SearchLocalPoints();

    // Optimize Pose
    int num_inliers = Optimizer::PoseOptimization(&mCurrentFrame,&mLastFrame);//motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();
//     Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();

    mnMatchesInliers = 0;
  PRINT_DEBUG_INFO_MUTEX( "num_inliers22=" << num_inliers <<endl, imu_tightly_debug_path, "debug.txt");
    // Update MapPoints Statistics
    set<MapPoint*> sMP;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
    for(int i=0; i<mCurrentFrame.N; i++) {
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
        } else if (mSensor == System::STEREO)  // why not include System::RGBD?
          mCurrentFrame.EraseMapPointMatch(i);
      }
    }

  PRINT_DEBUG_INFO_MUTEX( "inliers_map="<<mnMatchesInliers<<endl, imu_tightly_debug_path, "debug.txt");
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently (recent 1s)
    int threInlierReloc=50,threInliers=30;
    if (mbOnlyTracking&&mCurrentFrame.GetEncPreInt().mdeltatij>0){//rectified like TrackLocalMapWithIMU() for fusion effect (it largely affects Localization Mode)
      threInlierReloc=25;
      threInliers=15;
    }
    threInliers=15;//TODO: check
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<threInlierReloc)
        return false;

    if(mnMatchesInliers<threInliers)//notice it's a class data member
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)//the settings fps used here, if at initial step add new KF quickly while at relocalisation step add it slowly
        return false;

    // Do not insert keyframes if bias is not computed in VINS mode, maybe we can change localBA to pure-vision when mbRelocBiasPrepare=true and create a thread to RecomputeIMUBiasAndCurrentNavstate() like IMU Initialization!
    if(mbRelocBiasPrepare) return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2||mState==OK&&mpLastKeyFrame->mnId<mnLastOdomKFId+2)//just check for one(with ur>=0) KF demand for RGBD, if one of former 2 KFs is ODOMOK must also use this like nKFs<=2!!!
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);//the number of good MinObs(for Monocular) KFs tracked MapPoints in the RefKF

    // check if Local Mapping accept keyframes or LM thread is idle
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
      const auto& curfmps = mCurrentFrame.GetMapPointMatches();
      if (!mCurrentFrame.mv3Dpoints.size()) {
        for (int i = 0; i < mCurrentFrame.N; i++) {
          if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)  // it's a close point
          {
            if (curfmps[i] && !mCurrentFrame.mvbOutlier[i])  // it's a inlier map point or tracked one
              nTrackedClose++;
            else
              nNonTrackedClose++;
          }
        }
      } else {
        for (int k = 0; k < mCurrentFrame.mv3Dpoints.size(); ++k) {
          if (mCurrentFrame.goodmatches_[k]) {
            size_t i = mCurrentFrame.mapidxs2n_[k];
            CV_Assert(-1 != i);
            float z = mCurrentFrame.mvDepth[i];
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

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);//τt=100(enough dis)( τc=70(enough info) for stereo/RGBD to insert a new KF

    double timegap = 0.5;
//     if (!mpIMUInitiator->GetVINSInited()) timegap=0.1;//JW uses different timegap during IMU Initialization(0.1s)
    bool cTimeGap =false;int minClose=70;
    if (mpIMUInitiator->GetSensorIMU()) {
      cTimeGap = ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= timegap) && bLocalMappingIdle &&
                 mnMatchesInliers > 15;
      //     if (mpIMUInitiator->GetVINSInited()){//also we can use GetSensorIMU()
      bNeedToInsertClose = false;  // for VIO+Stereo/RGB-D, we don't open this inerstion strategy for speed and cTimeGap can do similar jobs
      if (mState == ODOMOK) {  // for VIEO+RGB-D, cTimeGap won't affect ODOMOK, so we may need it
        cTimeGap = ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= timegap) && bLocalMappingIdle;
      }
      //       minClose=100;
    }

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2||mState==OK&&mpLastKeyFrame->mnId<mnLastOdomKFId+2)//it's necessary for this stricter enough distance threshold! in my dataset Corridor004, like nKFs<=2!
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;//JingWang uses 0.8f

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;//time span too long(1s)
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);//for minF=0, if LocalMapper is idle
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;//rgbd/stereo tracking weak outside(large part are far points)
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);//not too close && not too far

    //Condition 3: odom && time conditon && min new close points' demand
    const bool c3=(mState==ODOMOK)&&(c1a||c1b||c1c)&&nNonTrackedClose>70;//may we can also use &&mCurrentFrame.N>500 like StereoInitialization()

    if((c1a||c1b||c1c)&&c2||cTimeGap||c3)//cTimeGap added by JingWang
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else//it wating queue is too long still refuse to insert new KFs
                    return false;
            }//it it's monocular, LocalMapper not idle -> refuse to insert new KFs
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame(cv::Mat img[2])
{
    if(!mpLocalMapper->SetNotStop(true))//if localMapper is stopped by loop closing thread/GUI, cannot add KFs; during adding process, it cannot be stopped by others
        return;

    //ensure Tcw is always right for mCurrentFrame even there's no odom data, NavState/Tbw can be wrong when there's no odom data
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB,mpLastKeyFrame,mState);//copy initial Tcw&Tbw(even wrong), update bi=bi+dbi
    //here notice a fact: JingWang haven't considered no imu data's condition when imu is initialized(including reloc.), so his NavState after imu intialized is always right, but before is also wrong, but he should set the right NavState for the first initialized KeyFrame
    //so I need to UpdateNavStatePVRFromTcw for the Frame when imu data is empty after imu is initialized
    if (mState==ODOMOK) mnLastOdomKFId=pKF->mnId;

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    PreIntegration(2);//zzh, though it doesn't need to be calculated when IMU isn't initialized

    if(mSensor!=System::MONOCULAR) {
        if (img) {
            pKF->Img[0] = img[0];
            pKF->Img[1] = img[1];
        }//zzh for PCL map creation

        mCurrentFrame.UpdatePoseMatrices();//UnprojectStereo() use mRwc,mOw, maybe useless

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest close points
        vector<pair<float, int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        if (!mCurrentFrame.mv3Dpoints.size()) {
          for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
              vDepthIdx.push_back(make_pair(z, i));
            }
          }
        } else { // TODO: speed up code, which could be unified with that in NeedNewKeyFrame()
          for (int k = 0; k < mCurrentFrame.mv3Dpoints.size(); ++k) {
            if (mCurrentFrame.goodmatches_[k]) {
              int i = mCurrentFrame.mapidxs2n_[k];
              float z = mCurrentFrame.mvDepth[i];
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
                vector<MapPoint *> pmps;
                if (-1 == ididxs) {
                  MapPoint* pMP = curfmps[i];
                  if (!pMP)
                    bCreateNew = true;
                  else if (pMP->Observations() < 1) {
                    bCreateNew = true;
                    CV_Assert(mbOnlyTracking || pMP->isBad());
                    //is there memory leak? we could use shared_ptr to solve this problem
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
                      if (pMP && !pMP->isBad()) { // we don't replace old good mps with stereo pt
                        bCreateNew = false;
                        //break; //use this when no pmps check
                      }
                      pmps.push_back(pMP);
                    }
                  }
                }

                if (bCreateNew) {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                    size_t ididxs = mCurrentFrame.GetMapn2idxs(i);
                    PRINT_DEBUG_INFO_MUTEX("mp0["<<pKF->mnId<<","<<i<<"]:", imu_tightly_debug_path, "debug.txt");
                    if (-1 == ididxs) {
                      pNewMP->AddObservation(pKF, i);
                      pKF->AddMapPoint(pNewMP, i);
                      mCurrentFrame.AddMapPoint(pNewMP, i);
                    } else {
                      auto idxs = mCurrentFrame.mvidxsMatches[ididxs];
                      bool icheck = false;
                      CV_Assert(idxs.size());
                      for (int cami = 0; cami < idxs.size(); ++cami) {
                        if (-1 != idxs[cami]) {
                          auto icami = mCurrentFrame.mapin2n_[cami][idxs[cami]];
                          if (icami == i) icheck = true;
                          pNewMP->AddObservation(pKF, icami);
                          pKF->AddMapPoint(pNewMP, icami);
                          mCurrentFrame.AddMapPoint(pNewMP, icami);
                        }
                      }
                      PRINT_DEBUG_INFO_MUTEX(endl, imu_tightly_debug_path, "debug.txt");
                      CV_Assert(icheck);
                    }
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);
                    nPoints++;
                } else {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth &&
                    nPoints > 100)//&&mState==OK)//maybe we can also use this for ODOMOK like StereoInitialization()
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;

    mbKeyFrameCreated=true;//for ros_mono_pub.cc
}

void Tracking::SearchLocalPoints()
{
  // Do not search map points already matched (in TrackWithMotionModel()/...), \
  all of (these) map points created by CreateNewKeyFrame()/StereoInitialization()/{UpdateLastFrame()in Localization mode/\
  CreateNewMapPoints() in LocalMapping}
  size_t num_maps_ready = 0;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
  for(size_t i = 0; i < curfmps.size(); ++i) {
    MapPoint* pMP = curfmps[i];
    if (pMP) {
      if (pMP->isBad()) {
        mCurrentFrame.EraseMapPointMatch(i);
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;  // don't need to match it in SBP()
        size_t n_cams = pMP->vbtrack_inview.size();
        pMP->vbtrack_inview.clear();
        pMP->vbtrack_inview.resize(n_cams, false);
        ++num_maps_ready;
      }
    }
  }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)//jump the already in-mCurrentFrame.mvpMapPoints MapPoints
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching,like mbTrackInView=true...)
	//judge if mCurrentFrame's centre is in the effective descriptor area(scale&&rotation invariance) of the MapPoint(with best descriptor&&normalVector)
        if(mCurrentFrame.isInFrustum(pMP,0.5))//no problem for mRcw,mtcw for mCurrentFrame.SetPose() in TrackWithMotionModel()/TrackReferenceKeyFrame()
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }
    PRINT_DEBUG_INFO_MUTEX("extra init=" << nToMatch << endl, imu_tightly_debug_path, "debug.txt");

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);//0.8 is the threshold for mindist/mindist2(<th is nice matching)
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        // th=10; // ORB3 use 10 here for imu_stereo
        nToMatch = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);//rectify the mCurrentFrame.mvpMapPoints
    }
    PRINT_DEBUG_INFO_MUTEX("befopt2 extra=" << nToMatch << endl, imu_tightly_debug_path, "debug.txt");
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization,but visualized MapPoints are the last F ones
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)//current F visible MapPoints initial mnTrackReferenceForFrame==0(mCurrentFrame.mnId entering this func. cannot be 0)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;//so it's for avoiding redundant addition
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
  const auto& curfmps = mCurrentFrame.GetMapPointMatches();
    for(int i=0; i<mCurrentFrame.N; i++) {
      if (curfmps[i]) {
        MapPoint* pMP = curfmps[i];
        if (!pMP->isBad()) {
          const auto observations = pMP->GetObservations();
          for (map<KeyFrame*, set<size_t>>::const_iterator it = observations.begin(), itend = observations.end();
               it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          mCurrentFrame.EraseMapPointMatch(i);
        }
      }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);//looser than covisibility graph demand
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;//to avoid repetition when selecting neighbor KFs(2nd layer covisible KFs&& neighbors of the spanning tree)
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)//avoid for replicated push_back for different itKF, this cannot be mCurrentFrame(not KF now)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)//still maybe rectify this two refKF variables in CreateNewKeyFrame()
    {
        mpReferenceKF = pKFmax;//highest/max covisible weight/MPs KF
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);//similar threshold in TrackReferenceKeyFrame()

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)//same threshold in TrackReferenceKeyFrame()
            {
                vbDiscarded[i] = true;
                continue;//useless
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);//get transformation by 3D-2D matches
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reaches max. iterations discard keyframe
            if(bNoMore)//to avoid too long time in RANSAC, at most 300 iterations from while start here
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.GetTcwRef());

                set<MapPoint*> sFound;

                const int np = vbInliers.size();//vvpMapPointMatches[i].size()/mCurrentFrame.mvpMapPoints.size()

                for(int j=0; j<np; j++) {
                  if (vbInliers[j]) {
                    mCurrentFrame.ReplaceMapPointMatch(j, vvpMapPointMatches[i][j]);
                    sFound.insert(vvpMapPointMatches[i][j]);
                  } else
                    mCurrentFrame.EraseMapPointMatch(j);
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);//use RANSAC P4P to select good inlier 3D-2D matches, then use all these matches to motion-BA

                if(nGood<10)//without checking pMP->Observation(), the number of inliers by motion-BA, the same threshold as TrackWithMotionModel()/TrackReferenceKeyFrame()
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)//delete outliers
                    if(mCurrentFrame.mvbOutlier[io])
                      mCurrentFrame.EraseMapPointMatch(io);

                // If few inliers, search by projection in a coarse or fine window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);//using additional matches to motion-BA

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                          const auto& curfmps = mCurrentFrame.GetMapPointMatches();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(curfmps[ip])//include outliers of just former BA for nGood isn't changed or don't believe former wide SBP() BA
                                    sFound.insert(curfmps[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                      mCurrentFrame.EraseMapPointMatch(io);
                            }
                        }//why don't delete mCurrentFrame.mvbOutlier when nGood>=50?
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)//the same threshold as TrackLocalMap() in Relocalization mode
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;

	if (!mbOnlyTracking&&mpIMUInitiator->GetSensorIMU()&&!mpIMUInitiator->mbUsePureVision){//Tracking mode doesn't enter this part
	  assert(mpIMUInitiator->GetVINSInited()&&"VINS not inited? why.");
	  mbRelocBiasPrepare=true;//notice we should call RecomputeIMUBiasAndCurrentNavstate() when 20-1 frames later, see IV-E in VIORBSLAM paper
	}
        return true;
    }

}

void Tracking::Reset()
{
    PRINT_INFO_MUTEX( "System Reseting" << endl);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    PRINT_INFO_MUTEX( "Reseting Local Mapper...");
    mpLocalMapper->RequestReset();
    PRINT_INFO_MUTEX( " done" << endl);

    // Reset Loop Closing
    PRINT_INFO_MUTEX( "Reseting Loop Closing...");
    mpLoopClosing->RequestReset();
    PRINT_INFO_MUTEX( " done" << endl);

    //zzh: Reset IMU Initialization, must after mpLocalMapper&mpLoopClosing->RequestReset()! for no updation of mpCurrentKeyFrame& no use of mbVINSInited in IMUInitialization thread
    cout<<"Resetting IMU Initiator...";mpIMUInitiator->RequestReset();cout<<" done"<<endl;
    mbRelocBiasPrepare=false;mnLastOdomKFId=0;
    mnLastRelocFrameId=0;

    // Clear BoW Database
    PRINT_INFO_MUTEX( "Reseting Database...");
    mpKeyFrameDB->clear();
    PRINT_INFO_MUTEX( " done" << endl);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    MapPoint::nNextId=0;//added by zzh
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    //for monocular!
    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    relative_frame_bvwbs_.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
    //added by zzh
    static bool bSwitch=false;//a switch for VIEO/VIO suitable tracking mode(VEO/RGBD) and VIEO/VIO 2nd/continuous SLAM mode
    if (flag){
      if (mpIMUInitiator->GetVINSInited()){
	bSwitch=true;
	mpIMUInitiator->SetVINSInited(false);//notice tracking mode only has RGBD/VEO, so VIEO uses VEO & VIO uses RGBD
      }
    }else{
      if (bSwitch){
	bSwitch=false;
	mpIMUInitiator->SetVINSInited(true);//notice tracking mode only has RGBD/VEO, so VIEO uses VEO & VIO uses RGBD
	mState=LOST;
      }
    }
}



} //namespace ORB_SLAM
