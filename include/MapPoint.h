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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace VIEO_SLAM
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
  void UpdateScale(const float &scale);
  
  //for Load/SaveMap()
  MapPoint(KeyFrame *pRefKF,Map *pMap,istream &is);
  bool read(istream &is);
  bool write(ostream &os);
  
//added by zzh
  
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);//used in localization mode tracking

    void SetWorldPos(const cv::Mat &Pos);//Pos.copyTo(mWorldPos)
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();//mNormalVector
    KeyFrame* GetReferenceKeyFrame();//mpRefKF, mpRefKF->mvpMapPoints should has this MP

    //if map is large, for single search time cost stability, please use map instead of unordered_map
    std::map<KeyFrame*,size_t> GetObservations();//mObservations
    int Observations();//nObs

    void AddObservation(KeyFrame* pKF,size_t idx);//mObservations[pKF]=idx;nObs+=2/1;
    void EraseObservation(KeyFrame* pKF);//mObservations.erase(pKF), update nObs and when nObs<=2 ->SetBadFlag()

    int GetIndexInKeyFrame(KeyFrame* pKF);//mObservations[pKF](-1 unfound)
    bool IsInKeyFrame(KeyFrame* pKF);//mObservations.count(pKF)

    void SetBadFlag();//mbBad=true && delete this MP/matches in this->mObservations.first(KFs) && mObservations.clear() && delete this MP in mpMap
    bool isBad();//mbBad

    void Replace(MapPoint* pMP);//SetBadFlag() && replace this MP by pMP in this->mObservations.first(KFs)
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();//Take the descriptor with least median distance to the rest

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();//0.8*mfMinDistance
    float GetMaxDistanceInvariance();//1.2*mfMaxDistance
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;//local BA in LocalMapping
    long unsigned int mnFuseCandidateForKF;//fuse in LocalMapping

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;//used in ComputeSim3() in LoopClosing
    long unsigned int mnCorrectedByKF;//avoid duplications, used in CorrectLoop()(&& PoseGraphOpt.) in LoopClosing 
    long unsigned int mnCorrectedReference;//ID of the KF which has corrected this MP's mWordPos, used in CorrectLoop() && PoseGraphOpt. in Optimizer.cc
    cv::Mat mPosGBA;//optimized Pos in the end of GBA
    long unsigned int mnBAGlobalForKF;//mpCurrentKF calling GBA


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction (not definitely normalized)
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame (the first KF in mObservations)
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;//number of Frames with visible matches in SearchLocalPoints() in Tracking
     int mnFound;//number of inliers in mnVisible in TrackLocalMap()/Track(for localization mode) in Tracking

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
