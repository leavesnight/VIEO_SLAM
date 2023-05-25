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
#include "MultiThreadBase.h"

namespace VIEO_SLAM {

class MapPoint;
class KeyFrame;

class Map : public MutexUsed {
  class KFIdComapre {  // ready for mspKeyFrames set less func., used in IMU Initialization thread, and I think it may
                       // help the insert speed
   public:
    bool operator()(const KeyFrame* kfleft, const KeyFrame* kfright) const;
  };
  int mnChangeIdx;  // Index related to any change when mMutexMapUpdate is locked && current KF's Pose is changed
 public:
  // for scale updation in IMU Initialization thread
  std::mutex mMutexScaleUpdateGBA, mMutexScaleUpdateLoopClosing;

  void InformNewChange() {
    unique_lock<std::mutex> lock(mMutexMap);
    ++mnChangeIdx;
  }
  int GetLastChangeIdx() {  // used for Tracking strategy choice
    unique_lock<mutex> lock(mMutexMap);
    return mnChangeIdx;
  }
  void ClearBadMPs();
  void clearMPs();

  // created by zzh over.

 public:
  Map();

  void AddKeyFrame(KeyFrame* pKF);
  void AddMapPoint(MapPoint* pMP);
  void EraseMapPoint(MapPoint* pMP);
  void EraseKeyFrame(KeyFrame* pKF);  // mspKeyFrames.erase(pKF)

  long unsigned KeyFramesInMap();
  std::vector<KeyFrame*> GetAllKeyFrames();  // vec(mspKeyFrames)
  std::vector<MapPoint*> GetAllMapPoints();  // vec(mspMapPoints)

  // for imu init
  int max_id_cam_ = 0, max_id_cam_unfixed_ = -1;  // we should ensure num_cam = max_id_cam_ + 1
  template <typename _Tp>
  using vector = std::vector<_Tp>;
  /* get lastKFs from newest one with >=time_span and neighbor KFs' delta_tij <= max_delta_tij
   * please give enough time_span_max redundant from time_span
   */
  vector<KeyFrame*> GetLastKFs(double time_span, vector<bool>& benough_id_cam, double time_span_max = 16,
                               double max_delta_tij = 3.0, const int8_t fix_mode = 2, const size_t min_num = 0);

  // mnBigChangeIdx++, for System::MapChanged(), notice map is changed even just
  // CorrectLoop() is run though GBA may be cancelled by a new loop
  void InformNewBigChange();
  int GetLastBigChangeIdx();  // used for System::MapChanged()

  void clear();

  long unsigned int GetMaxKFid();  // mnMaxKFid

  long unsigned int MapPointsInMap();                               // mspMapPoints.size()
  void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);  // mvpReferenceMapPoints = vpMPs
  std::vector<MapPoint*> GetReferenceMapPoints();

  std::vector<KeyFrame*> mvpKeyFrameOrigins;  // pushed pKFini in StereoInitialization() in Tracking for RGBD

  // update KFs' Pose and their mvpMapPoints' Pos and KF&&MP's relation(KF.mvpMapPoints&&MP.mObservations), used in
  // Track() && LocalBA in LocalMapping && initialize_imu && CorrectLoop()(&& SearchAndFuse()&&PoseGraphOpt.) in
  // LoopClosing && GBA thread
  std::mutex mMutexMapUpdate;

  // This avoid that two points are created simultaneously in separate threads (id conflict)
  std::mutex mMutexPointCreation;  // used in new MapPoint() in Tracking/LocalMapping thread

 protected:
  std::set<MapPoint*> mspMapPoints;
  std::set<KeyFrame*, KFIdComapre> mspKeyFrames;  // zzh, it's very important!

  std::vector<MapPoint*> mvpReferenceMapPoints;

  long unsigned int mnMaxKFid;

  // Index related to a big change in the map (loop closure, global BA)
  int mnBigChangeIdx;

  std::mutex mMutexMap;  // single variable updation mutex, while MapUpdate is the whole Map mutex
};

}  // namespace VIEO_SLAM

#endif  // MAP_H
