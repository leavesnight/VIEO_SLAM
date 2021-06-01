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


#ifndef TRACKING_H
#define TRACKING_H

#include "OdomData.h"
#include<chrono>//for delay control

//created by zzh over.

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace VIEO_SLAM
{
class IMUInitialization;//zzh, for they includes each other

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  
  //Created by zzh
  //pure encoder edges
  void TrackWithOnlyOdom(bool bMapUpdated);
  // Odom PreIntegration
  template<class EncData>
  inline bool iterijFind(const listeig(EncData) &mlOdomEnc,const double &curTime,
			 typename listeig(EncData)::const_iterator &iter,const double&errOdomImg,bool bSearchBack=true);
  template<class _OdomData>
  void PreIntegration(const char type,listeig(_OdomData) &mlOdom,
		      typename listeig(_OdomData)::const_iterator &miterLastEnc,Frame *pLastF=NULL,Frame *pCurF=NULL);//0 for initialize,1 for inter-Frame PreInt.,2 for inter-KF PreInt. \
  0/2 also is used to cull the data in 2 lists whose tm is (mLastKeyFrame.mTimeStamp,mCurrentKeyFrame.mTimeStamp], \
  culling strategy: tm<mtmSyncOdom is discarded & tm>mCurrentFrame.mTimeStamp is reserved in lists & the left is needed for deltax~ij calculation, \
  for the case Measurement_j-1 uses (M(tj)+M(tj-1))/2, we also reserved last left one in 2 lists(especially for Enc); \
  if pLastF & pCurF exit, we use them instead of mLastFrame & mCurrentFrame
  void PreIntegration(const char type=0);
  bool TrackWithIMU(bool bMapUpdated);//use IMU prediction instead of constant velocity/uniform motion model
  // Predict the NavState of Current Frame by IMU
  bool PredictNavStateByIMU(bool bMapUpdated);//use IMU motion model, like motion update/prediction in ekf, if prediction failed(e.g. no imu data) then false
  //IMUPreintegrator GetIMUPreIntSinceLastKF();
  bool TrackLocalMapWithIMU(bool bMapUpdated);//track local map with IMU motion-only BA, if no imu data it degenerates to TrackLocalMap()
  void RecomputeIMUBiasAndCurrentNavstate();//recompute bias && mCurrentFrame.mNavState when 19th Frame after reloc.
  
  // Get mVelocity by Encoder data
  bool GetVelocityByEnc(bool bMapUpdated=false);
  
  // Flags for relocalization. Create new KF once bias re-computed & flag for preparation for bias re-compute
  bool mbRelocBiasPrepare;//true means preparing/not prepared
  // 20 Frames are used to compute bias
  vector<Frame*> mv20pFramesReloc;//vector<Frame,Eigen::aligned_allocator<Frame> > mv20FramesReloc used by JW, notice why we must use Eigen::aligned_allocator(quaterniond in NavState, or Segementation fault)
  
  //Consts
  //Error allow between "simultaneous" IMU data(Timu) & Image's mTimeStamp(Timg): Timu=[Timg-err,Timg+err]
  double mdErrIMUImg;
  //Tbc,Tbo
  cv::Mat mTbc,mTce;//Tbc is from IMU frame to camera frame;Tbo is from IMU frame to encoder frame(the centre of 2 driving wheels, +x pointing to forward,+z pointing up)
  //delay time of Odom Data received(CacheOdom) relative to the Image entering(GrabImageX), or Camera.delayForPolling
  double mDelayCache;
  //delay time of the image's timestamp to odom's timestamp, or delaytoimu, delaytoenc: Timg=Todom+mDelayToOdom
  double mDelayToIMU,mDelayToEnc;
  //Variables
  std::chrono::steady_clock::time_point mtmGrabDelay;//for delay control(we found our Enc&IMU's response has some delay=20ms)
  //cache queue for vl,vr/IMU & its own timestamp from LastKF
  listeig(EncData) mlOdomEnc;
  listeig(IMUData) mlOdomIMU;
  std::mutex mMutexOdom;//for 2 lists' multithreads' operation
  listeig(EncData)::const_iterator miterLastEnc;//Last EncData pointer in LastFrame, need to check its tm and some data latter to find the min|mtmSyncOdom-tm| s.t. tm<=mtmSyncOdom
  listeig(IMUData)::const_iterator miterLastIMU;//Last IMUData pointer in LastFrame, we don't change the OdomData's content
  
  unsigned long mnLastOdomKFId;

public:
  //Add Odom(Enc/IMU) data to cache queue
  cv::Mat CacheOdom(const double &timestamp, const double* odomdata, const char mode);
   
  void SetLastKeyFrame(KeyFrame* pKF){
    mpLastKeyFrame=pKF;
  }
  void SetReferenceKF(KeyFrame* pKF){
    mpReferenceKF=pKF;
  }
     
   //for ros_mono_pub.cc
  bool mbKeyFrameCreated;
  cv::Mat GetKeyFramePose(){
    return mpReferenceKF->GetPose();
  }
   
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//created by zzh over.
  
public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor); 

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetIMUInitiator(IMUInitialization *pIMUInitiator){mpIMUInitiator=pIMUInitiator;}//zzh
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);//unused here

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,//used by FrameDrawer, not Tracking
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3,
        ODOMOK=4,//added by zzh
	MAP_REUSE=5,//added by zzh
	MAP_REUSE_RELOC=6//added by zzh
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;//value's type is VIEO_SLAM::System::eSensor

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;//used by FrameDrawer

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;//true for lost!

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:
    // Main tracking function. It is independent of the input sensor.
    void Track(cv::Mat img[2]=NULL);//img[2] recorded by KFs

    // Map initialization for stereo and RGB-D
    void StereoInitialization(cv::Mat img[2]=NULL);

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame(int thInMPs=10,int thMatch=15);//track mCurrentFrame with mpReferenceKF by SBB and motion-only BA(if nmatches is enough), then \
    discard outliers, return nInliers>=10
    void UpdateLastFrame();//update last Frame's Pose by reference KeyFrame&&mlRelativeFramePoses for nonlocalization mode
    bool TrackWithMotionModel();//UpdateLastFrame, use SBP to get mCurrentFrame.mvpMapPoints, then motion-only BA(if nmatches is enough), \
    discard outliers, return nInliers>=10 for nonlocalization mode

    bool Relocalization();

    void UpdateLocalMap();//mpMap->SetReferenceMapPoints(mvpLocalMapPoints), UpdateLocalKeyFrames&&UpdateLocalPoints
    void UpdateLocalPoints();//use mvpLocalKeyFrames[i]->mvpMapPoints to fill mvpLocalMapPoints(avoid duplications by pMP->mnTrackReferenceForFrame)
    void UpdateLocalKeyFrames();//use mCurrentFrame&&its covisible KFs(>=1 covisible MP)&&the KFs' neighbors(10 best covisibility KFs&&parent&&children) \
    to make mvpLocalKeyFrames, update (mCurrentFrame.)mpReferenceKF to max covisible KF

    bool TrackLocalMap();//use UpdateLocalMap&&SearchLocalPoints(mvpLocalMapPoints) to add new matched \
    mvpMapPoints in mCurrentFrame, then motion-only BA to add Pose's accuracy and update mnMatchesInliers&&pMP->mnFound, \
    finally may judge mnMatchesInliers to one ballot veto the mState to make it Lost
    void SearchLocalPoints();//update mCurrentFrame->mvpMapPoints(also discard bad ones)+mvpLocalMapPoints' pMP->mnVisible&&mnLastFrameSeen and \
    call mCurrentFrame.isInFrustum(pMP,0.5), then try to match mvpLocalMapPoints to mCurrentFrame by SBP()(add some mvpMapPoints)

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame(cv::Mat img[2]=NULL);

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;
    IMUInitialization* mpIMUInitiator;//zzh

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;//corresponding to mCurrentFrame(most of time ==mCurrentFrame.mpReferenceKF)
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;//40b, here TUM use 3.2(m)

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;//rectified in TrackLocalMap()

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

//created by zzh
template<class EncData>
bool Tracking::iterijFind(const listeig(EncData) &mlOdomEnc,const double &curTime,
			  typename listeig(EncData)::const_iterator &iter,const double&errOdomImg,bool bSearchBack){
  typename listeig(EncData)::const_iterator iter1;//iter-1 or iter+1
  double minErr;//=errOdomImg+1;
  if (bSearchBack){//should==mlOdomEnc.end()
    while (iter!=mlOdomEnc.begin()){//begin is min >= lastKFTime-err
      if ((--iter)->mtm<=curTime) break;//get max <= curTime+0
    }
    //Notice mlOdomEnc.empty()==false!
//     if (iter!=mlOdomEnc.end()){//find a nearest iter to curTime
      if (curTime>iter->mtm){//iter==begin may still have iter->mtm>curTime
        minErr=curTime-iter->mtm;
        iter1=iter;++iter1;//iter+1
        if (iter1!=mlOdomEnc.end()&&(iter1->mtm-curTime<minErr)){
          iter=iter1;
          minErr=iter1->mtm-curTime;
        }
      }else{//we don't need to compare anything when iter->mtm>=curTime
	    minErr=iter->mtm-curTime;
      }
//     }//==mlOdomEnc.end(), we cannot test ++iter
  }else{//should==mlOdomEnc.begin()
    while (iter!=mlOdomEnc.end()){//begin is min >= lastKFTime-err
      if (iter->mtm>=curTime) break;//get min >= lastKFTime+0
      ++iter;
    }
    if (iter!=mlOdomEnc.end()){
      minErr=iter->mtm-curTime;
      if (iter!=mlOdomEnc.begin()){
        iter1=iter;--iter1;//iter-1
        if (curTime-iter1->mtm<minErr){
          iter=iter1;
          minErr=curTime-iter1->mtm;
        }
      }
    }else{//==mlOdomEnc.end(), but we can test --iter
      //Notice mlOdomEnc.empty()==false!
      --iter;
      minErr=curTime-iter->mtm;
    }
  }
  if (minErr<=errOdomImg){//we found nearest allowed iterj/iteri to curTime/lastKFTime
    return true;
  }
  return false;//if iteri/j is not in allowed err, returned iter points to nearest one to curTime or end()
}
template<class EncData>
void Tracking::PreIntegration(const char type,listeig(EncData) &mlOdomEnc,
			      typename listeig(EncData)::const_iterator &miterLastEnc,Frame *pLastF,Frame *pCurF){
  switch (type){//0/2 will cull 2 Odom lists,1 will shift the pointer
    case 0://for 0th keyframe/frame: erase all the data whose tm<=mCurrentFrame.mTimeStamp but keep the last one, like list.clear()
      if (!mlOdomEnc.empty()){
        double curTime=mCurrentFrame.mTimeStamp;
        typename listeig(EncData)::const_iterator iter=mlOdomEnc.end();
        iterijFind<EncData>(mlOdomEnc,curTime,iter,mdErrIMUImg);//we just find the nearest iteri(for next time) to curTime, don't need to judge if it's true
        cout<<redSTR"ID="<<mCurrentFrame.mnId<<"; curDiff:"<<iter->mtm-curTime<<whiteSTR<<endl;

        mlOdomEnc.erase(mlOdomEnc.begin(),iter);//retain the nearest allowed EncData / iteri used to calculate the Enc PreIntegration
        miterLastEnc=mlOdomEnc.begin();//nearest iterj to curTime(next time it may not be the nearest iteri to lastKFTime when mlOdom.back().mtm<curTime); maybe end() but we handle it in the CacheOdom()
      }
      break;
    case 1:
      //PreIntegration between 2 frames, use pLastF & pCurF to be compatible with RecomputeIMUBiasAndCurrentNavstate()
      if (!mlOdomEnc.empty()){
        if (pLastF==NULL) pLastF=&mLastFrame;if (pCurF==NULL) pCurF=&mCurrentFrame;
        double curFTime=pCurF->mTimeStamp,lastFTime=pLastF->mTimeStamp;
        typename listeig(EncData)::const_iterator iter=mlOdomEnc.end();
        if (iterijFind<EncData>(mlOdomEnc,curFTime,iter,mdErrIMUImg)&&iterijFind<EncData>(mlOdomEnc,lastFTime,miterLastEnc,mdErrIMUImg,false)){//iterj&iteri both found then calculate delta~xij(phi,p)
    // 	  assert((miterLastEnc->mtm-lastFTime)==0&&(iter->mtm-curFTime)==0);
    // 	  cout<<redSTR"ID="<<pCurF->mnId<<"; LastDiff:"<<miterLastEnc->mtm-lastFTime<<", curDiff:"<<iter->mtm-curFTime<<whiteSTR<<endl;
          pCurF->PreIntegration<EncData>(pLastF,miterLastEnc,iter);//it is optimized without copy
        }

        if (iter!=mlOdomEnc.end())
          miterLastEnc=iter;//update miterLastEnc pointing to the nearest(now,not next time) one of this frame / begin for next frame
        else
          miterLastEnc=--iter;//if not exist please don't point to the end()! so we have to check the full restriction of miterLastX, it cannot be --begin() for !mlOdomEnc.empty()
      }
      break;
    case 2:
      //PreIntegration between 2 KFs & cull 2 odom lists: erase all the data whose tm<=mpReferenceKF(curKF)->mTimeStamp but keep the last one
      if (!mlOdomEnc.empty()){
        double curTime=mpReferenceKF->mTimeStamp,lastKFTime=mpLastKeyFrame->mTimeStamp;
        typename listeig(EncData)::const_iterator iter=mlOdomEnc.end(),iteri=mlOdomEnc.begin();//iterj, iteri
        if (iterijFind<EncData>(mlOdomEnc,curTime,iter,mdErrIMUImg)&&iterijFind<EncData>(mlOdomEnc,lastKFTime,iteri,mdErrIMUImg,false)){//iterj&iteri both found then calculate delta~xij(phi,p)
    // 	  assert((iteri->mtm-lastKFTime)==0&&(iter->mtm-curTime)==0);
    // 	  cout<<redSTR"ID="<<mCurrentFrame.mnId<<"; LastDiff:"<<iteri->mtm-lastKFTime<<", curDiff:"<<iter->mtm-curTime<<whiteSTR<<" LastTime="<<fixed<<setprecision(9)<<lastKFTime<<"; cur="<<mCurrentFrame.mTimeStamp<<endl;
          mpReferenceKF->SetPreIntegrationList<EncData>(iteri,iter);//save odom data list in curKF for KeyFrameCulling()
	    }
	
        mlOdomEnc.erase(mlOdomEnc.begin(),iter);//retain the nearest allowed EncData / iteri used to calculate the Enc PreIntegration
        miterLastEnc=mlOdomEnc.begin();//nearest iterj to curTime(next time it may not be the nearest iteri to lastKFTime when mlOdom.back().mtm<curTime); maybe end() but we handle it in the CacheOdom()

        mpReferenceKF->PreIntegration<EncData>(mpLastKeyFrame);//mpLastKeyFrame cannot be bad here for mpReferenceKF hasn't been inserted (SetBadFlag only for before KFs)
      }
      break;
    case 3:
      //PreIntegration between lastKF & curF
      if (!mlOdomEnc.empty()){
        double curFTime=mCurrentFrame.mTimeStamp,lastKFTime=mpLastKeyFrame->mTimeStamp;
        typename listeig(EncData)::const_iterator iter=mlOdomEnc.end(),iteri=mlOdomEnc.begin();
        if (iterijFind<EncData>(mlOdomEnc,curFTime,iter,mdErrIMUImg)&&iterijFind<EncData>(mlOdomEnc,lastKFTime,iteri,mdErrIMUImg,false)){//iterj&iteri both found then calculate delta~xij(phi,p)
    // 	  assert((iteri->mtm-lastKFTime)==0&&(iter->mtm-curFTime)==0);
    // 	  cout<<redSTR"ID="<<mCurrentFrame.mnId<<"; LastDiff:"<<iteri->mtm-lastKFTime<<", curDiff:"<<iter->mtm-curFTime<<whiteSTR<<endl;
          mCurrentFrame.PreIntegration<EncData>(mpLastKeyFrame,iteri,iter);//it is optimized without copy, Notice here should start from last KF!
        }

        if (iter!=mlOdomEnc.end())
          miterLastEnc=iter;//update miterLastEnc pointing to the nearest(now,not next time) one of this frame / begin for next frame
        else
          miterLastEnc=--iter;//if not exist please don't point to the end()! so we have to check the full restriction of miterLastX, it cannot be --begin() for !mlOdomEnc.empty()
      }
      break;
  }
}

} //namespace ORB_SLAM

#endif // TRACKING_H
