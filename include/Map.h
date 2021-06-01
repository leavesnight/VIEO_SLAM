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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace VIEO_SLAM
{

class MapPoint;
class KeyFrame;

class Map
{
  class KFIdComapre{//ready for mspKeyFrames set less func., used in IMU Initialization thread, and I think it may help the insert speed
  public:
    bool operator()(const KeyFrame* kfleft,const KeyFrame* kfright) const;
  };
  int mnChangeIdx;// Index related to any change when mMutexMapUpdate is locked && current KF's Pose is changed
  
public:
  //for scale updation in IMU Initialization thread
  std::mutex mMutexScaleUpdateGBA,mMutexScaleUpdateLoopClosing;
  
  void InformNewChange(){
    unique_lock<std::mutex> lock(mMutexMap);
    ++mnChangeIdx;
  }
  int GetLastChangeIdx(){//used for Tracking strategy choice
    unique_lock<mutex> lock(mMutexMap);
    return mnChangeIdx;
  }
  void ClearBadMPs();
  void clearMPs();
  
//created by zzh over.
  
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);//mspKeyFrames.erase(pKF)
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);//mvpReferenceMapPoints = vpMPs
    void InformNewBigChange();//mnBigChangeIdx++, for System::MapChanged(), notice map is changed even just CorrectLoop() is run though GBA may be cancelled by a new loop
    int GetLastBigChangeIdx();//used for System::MapChanged()

    std::vector<KeyFrame*> GetAllKeyFrames();//vec(mspKeyFrames)
    std::vector<MapPoint*> GetAllMapPoints();//vec(mspMapPoints)
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();//mspMapPoints.size()
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();//mnMaxKFid

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;//pushed pKFini in StereoInitialization() in Tracking for RGBD

    //update KFs' Pose and their mvpMapPoints' Pos and KF&&MP's relation(KF.mvpMapPoints&&MP.mObservations), \
    // used in Track() && LocalBA in LocalMapping && initialize_imu &&
    // CorrectLoop()(&& SearchAndFuse()&&PoseGraphOpt.) in LoopClosing && GBA thread
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;//used in new MapPoint() in Tracking/LocalMapping thread

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*,KFIdComapre> mspKeyFrames;//zzh, it's very important!

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;//single variable updation mutex, while MapUpdate is the whole Map mutex
};

} //namespace ORB_SLAM

#endif // MAP_H
