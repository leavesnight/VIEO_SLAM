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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "IMUInitialization.h"//zzh

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
//#include "KeyFrameDatabase.h"//unused

#include <mutex>


namespace VIEO_SLAM
{
  
class IMUInitialization;//zzh, for they includes each other

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
  unsigned long mnLastOdomKFId;
  KeyFrame* mpLastCamKF;
  
  //Local Window size
  int mnLocalWindowSize;//default 10, JW uses 20
  
//created by zzh over.
  
public:
    LocalMapping(Map* pMap, const bool bMonocular,const string &strSettingPath);//should use bool here

    void SetLoopCloser(LoopClosing* pLoopCloser);
    void SetTracker(Tracking* pTracker);
    void SetIMUInitiator(IMUInitialization *pIMUInitiator){mpIMUInitiator=pIMUInitiator;}//zzh

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);//mlNewKeyFrames.push_back(pKF) and mbAbortBA=true(stop localBA), \
    if use ,const char state=2: we cannot use Traking::OK/eTrackingState here for Tracking.h and LocalMapping.h include each other

    // Thread Synch
    void RequestStop();//non-blocking request stop, it will finally be stopped when it's idle, used in localization mode/CorrectLoop() in LoopClosing thread
    void RequestReset();//blocking(3ms refreshing) mode
    bool Stop();//try to stop when requested && allowed to be stopped
    void Release();//used in mbDeactivateLocalizationMode/CorrectLoop() in LoopClosing
    bool isStopped();//mbStopped
    bool stopRequested();
    bool AcceptKeyFrames();//if accept KFs, mbAcceptKeyFrames
    void SetAcceptKeyFrames(bool flag);//mbAcceptKeyFrames=flag
    bool SetNotStop(bool flag);//true means it cannot be stopped by others

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();//check if New KFs exit (!mlNewKeyFrames.empty())
    void ProcessNewKeyFrame();//calculate BoW,update mlNewKeyFrames&&mlpRecentAddedMapPoints(RGBD)&&MapPoints' normal&&descriptor, update connections in covisibility graph&& spanning tree, insert KF in mpMap
    void CreateNewMapPoints();//match CurrentKF with neighbors by BoW && validated by epipolar constraint,\
    triangulate the far/too close points by Linear Triangulation Method/depth data, then check it through positive depth, projection error(chi2 distri.) && scale consistency,\
    finally update pMP infomation(like mObservations,normal,descriptor,insert in mpMap,KFs,mlpRecentAddedMapPoints)

    void MapPointCulling();//delete some bad && too long ago MapPoints in mlpRecentAddedMapPoints
    void SearchInNeighbors();//find 2 layers(10,5) of neighbor KFs in covisibility graph, bijection search matches in neighbors and mpCurrentKeyFrame then fuse them,\
    update pMP's normal&&descriptor and CurrentKF's connections in covisibility graph

    void KeyFrameCulling();//erase redundant localKFs(all 1st layer covisibility KFs), redundant means 90% close stereo MPs seen by other >=3 KFs in same/finer scale

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);//calculate Fundamental Matrix F12=K1^(-T)*t12^R12*K2^(-1)

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);//calculate the v^=[0 -c b;c 0 -a;-b a 0]

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();//mbFinishRequested
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    IMUInitialization* mpIMUInitiator;//zzh
    Tracking* mpTracker;//unused

    
    std::list<KeyFrame*> mlNewKeyFrames;
    //std::list<Tracking::eTrackingState> mlNewKFStates;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;
    

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
