//created by zzh, inspired by JingWang
#ifndef IMUINITIALIZATION_H
#define IMUINITIALIZATION_H

//zzh defined color cout, must after include opencv2
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

#include <unistd.h>

namespace VIEO_SLAM{

class KeyFrame;
class Map;
class LocalMapping;

//notice Get##Name() calls copy constructor when return
#define CREATOR_VAR_MUTEX(Name,Type,Suffix) \
  Type m##Suffix##Name;\
  std::mutex mMutex##Name;
#define CREATOR_GET(Name,Type,Suffix) \
  Type Get##Name(void){\
    unique_lock<mutex> lock(mMutex##Name);\
    return m##Suffix##Name;\
  }
#define CREATOR_SET(Name,Type,Suffix) \
  void Set##Name(Type value){\
    unique_lock<mutex> lock(mMutex##Name);\
    m##Suffix##Name=value;\
  }
#define CREATOR_VAR_MULTITHREADS(Name,Type,Suffix) \
private:\
  CREATOR_VAR_MUTEX(Name,Type,Suffix)\
public:\
  CREATOR_GET(Name,Type,Suffix)\
  CREATOR_SET(Name,Type,Suffix)\
private:
  
using namespace Eigen;
using namespace std;

    class IMUKeyFrameInit;

class IMUInitialization{//designed for multi threads
  string mTmpfilepath;
  double mdInitTime,mdFinalTime;
  unsigned int mnSleepTime;
  double mdStartTime;//for reset
  //cv::Mat mRwiInit;//unused
  
  CREATOR_VAR_MULTITHREADS(SensorEnc,bool,b);
  CREATOR_VAR_MULTITHREADS(SensorIMU,bool,b);//for auto reset judgement of this system, automatically check if IMU exists, for it needs initialization with a quite long period of tracking without LOST
  CREATOR_VAR_MULTITHREADS(VINSInited,bool,b)//if IMU initialization is over
  cv::Mat mGravityVec; // gravity vector in world frame
  std::mutex mMutexInitIMU;//for mGravityVec, improved by zzh
  //double mnVINSInitScale; //scale estimation for Mono, not necessary here
  
  CREATOR_VAR_MULTITHREADS(CopyInitKFs,bool,b)//for copying/cache KFs in IMU initialization thread avoiding KeyFrameCulling()
  
  //CREATOR_VAR_MULTITHREADS(UpdatingInitPoses,bool,b)//for last propagation in IMU Initialization to stop adding new KFs in Tracking thread, useless for LocalMapping is stopped
  CREATOR_VAR_MULTITHREADS(InitGBA,bool,b)//for last GBA(include propagation) required by IMU Initialization, LoopClosing always creates new GBA thread when it's true
  CREATOR_VAR_MULTITHREADS(InitGBAOver,bool,b)//for 1st Full BA strategy Adjustment
  
  //like the part of LocalMapping
  CREATOR_VAR_MULTITHREADS(CurrentKeyFrame,KeyFrame*,p)//updated by LocalMapping thread
  CREATOR_VAR_MUTEX(Finish,bool,b)//checked/get by System.cc
  CREATOR_VAR_MUTEX(FinishRequest,bool,b)//requested/set by System.cc
  CREATOR_VAR_MULTITHREADS(Reset,bool,b)//for reset Initialization variables
  //const
  Map* mpMap;
  bool mbMonocular;
  LocalMapping* mpLocalMapper;//for Stop LocalMapping thread&&NeedNewKeyFrame() in Tracking thread
  
  bool TryInitVIO(void);
  bool TryInitVIO_zzh(void);
  cv::Mat SkewSymmetricMatrix(const cv::Mat &v){
      return (cv::Mat_<float>(3,3)<<0, -v.at<float>(2), v.at<float>(1),
				    v.at<float>(2), 0, -v.at<float>(0),
				    -v.at<float>(1), v.at<float>(0), 0);
  }
  void ResetIfRequested(){
    if(GetReset()){
      //reset relevant variables
      mdStartTime=-1;SetCurrentKeyFrame(NULL);//SetSensorIMU(false);
      SetVINSInited(false);//usually this 3 variables are false when LOST then this func. will be called
      SetInitGBA(false);//if it's true, won't be automatically reset
      SetInitGBAOver(false);
      
      SetReset(false);
    }
  }
  CREATOR_SET(Finish,bool,b)
  CREATOR_GET(FinishRequest,bool,b)
public:
  bool mbUsePureVision;//for pure-vision+IMU Initialization mode!
  
  IMUInitialization(Map* pMap,const bool bMonocular,const string& strSettingPath):mpMap(pMap),mbMonocular(bMonocular),
    mbFinish(true),mbFinishRequest(false),mbReset(false),verbose(0){
    mbSensorEnc=false;
    mdStartTime=-1;mbSensorIMU=false;mpCurrentKeyFrame=NULL;
    mbVINSInited=false;
    mbCopyInitKFs=false;
    mbInitGBA=false;mbInitGBAOver=false;
    
    cv::FileStorage fSettings(strSettingPath,cv::FileStorage::READ);
    cv::FileNode fnStr=fSettings["test.InitVIOTmpPath"];
    if (!fnStr.empty()) fnStr>>mTmpfilepath;
    else cout<<"Nothing recorded for analysis!"<<endl;
    //load mbUsePureVision
    cv::FileNode fnSize=fSettings["LocalMapping.LocalWindowSize"];
    if (fnSize.empty()){
      mbUsePureVision=true;
      cout<<redSTR"No LocalWindowSize, then don't enter VIORBSLAM2 or Odom(Enc/IMU) mode!"<<whiteSTR<<endl;
    }else{
      if ((int)fnSize<1){
	mbUsePureVision=true;
	cout<<blueSTR"mnLocalWindowSize<1, we use pure-vision+IMU Initialization mode!"<<whiteSTR<<endl;
      }else mbUsePureVision=false;
    }
    cv::FileNode fnTime[3]={fSettings["IMU.InitTime"],fSettings["IMU.SleepTime"],fSettings["IMU.FinalTime"]};
    if (fnTime[0].empty()||fnTime[1].empty()||fnTime[2].empty()){
      mdInitTime=0;mnSleepTime=1e6;mdFinalTime=15;
      cout<<redSTR"No IMU.InitTime&SleepTime&FinalTime, we use default 0s & 1s & 15s!"<<whiteSTR<<endl;
    }else{
      mdInitTime=fnTime[0];
      mnSleepTime=(double)fnTime[1]*1e6;
      mdFinalTime=fnTime[2];
    }
  }
  
  void Run();
  
  cv::Mat GetGravityVec(void);
  void SetGravityVec(const cv::Mat &mat);
  
  CREATOR_GET(Finish,bool,b)
  CREATOR_SET(FinishRequest,bool,b)
  void RequestReset(){//blocking(3ms refreshing) mode, called by Tracking thread
    SetReset(true);
    for(;;){
      if (!GetReset()) break;//if mbReset changes from true to false, resetting is finished
      usleep(3000);
    }
  }
  void SetLocalMapper(LocalMapping* pLocalMapper){
    mpLocalMapper=pLocalMapper;
  }
    static int deleteKFs_ret(vector<vector<IMUKeyFrameInit *>*> &vKFsInit);
    static int reduceKFs(const vector<char> &reduced_hids,
                         vector<vector<IMUKeyFrameInit *>*> &vKFsInit, vector<vector<IMUKeyFrameInit *>*> &vKFsInit2,
                         vector<int> &Ns, int &num_handlers, vector<char> &id_cams, vector<char> *id_cams_ref);

    char verbose;
};

class IMUKeyFrameInit{//a simple/base version of KeyFrame just used for IMU Initialization, not designed for multi threads
public://I think it belongs FramePoseBase
  const double mTimeStamp;//for ComputePreInt
  cv::Mat mTwc,mTcw;//for TryInitVIO()&OptimizeInitialGyroBias(),see (9) in VIORBSLAM paper
  //we don't save mTbc for it's constant
  
public:
  Vector3d mbg_,mba_;//bgj_bar,baj_bar: if changed, mIMUPreInt needs to be recomputed; unoptimized part of current defined mNavState
  IMUPreintegrator mOdomPreIntIMU;//including mlIMUData, for OptimizeInitialGyroBias()
  IMUKeyFrameInit* mpPrevKeyFrame;//but it's important for mOdomPreIntIMU computation && KeyFrameCulling()

  IMUKeyFrameInit(KeyFrame& kf);
  
  void ComputePreInt(){//0th frame don't use this function, mpPrevKeyFrame shouldn't be bad
    if (mpPrevKeyFrame==NULL) return;
#ifndef TRACK_WITH_IMU
    mOdomPreIntIMU.PreIntegration(mpPrevKeyFrame->mTimeStamp,mTimeStamp);
#else
    mOdomPreIntIMU.PreIntegration(mpPrevKeyFrame->mTimeStamp,mTimeStamp,mpPrevKeyFrame->mbg_,mpPrevKeyFrame->mba_);
#endif
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW//for maybe quaterniond in IMUPreintegrator
};

}

typedef enum kVerboseLevel{
    kVerbRel,
    kVerbDeb,
    kVerbFull
};

#endif
