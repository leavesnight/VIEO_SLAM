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

#include "Map.h"

#include <mutex>

namespace VIEO_SLAM {

bool Map::KFIdComapre::operator()(const KeyFrame *kfleft, const KeyFrame *kfright) const {  // zzh
  return kfleft->mnId < kfright->mnId;
}
void Map::ClearBadMPs() {
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints.clear();
  for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++) {
    if ((*sit)->isBad()) {
      delete *sit;
      mspMapPoints.erase(*sit);
    }
  }
}
void Map::clearMPs() {
  for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++) delete *sit;
  mspMapPoints.clear();
  mvpReferenceMapPoints.clear();
}

// created by zzh over

Map::Map()
    : mnMaxKFid(0),
      mnBigChangeIdx(0),
      mnChangeIdx(0)  // zzh
{}

void Map::AddKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.insert(pKF);
  if (pKF->mnId > mnMaxKFid) mnMaxKFid = pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP) {
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.erase(pKF);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame *> Map::GetLastKFs(double time_span, vector<bool> &benough_id_cam, double time_span_max,
                                   double max_delta_tij, const int8_t fix_mode, const size_t min_num) {
  unique_lock<mutex> lock(mMutexMap);
  auto iter = mspKeyFrames.end();
  auto iter_1 = iter--;
  double time_start = (*mspKeyFrames.begin())->timestamp_;
  int num_cam = fix_mode ? max_id_cam_ + 1 : max_id_cam_unfixed_ + 1;
  vector<double> time_end(num_cam, time_start);
  if (2 == fix_mode && (*iter)->timestamp_ - time_start <= time_span_max && !min_num) {
    benough_id_cam.clear();
    set<int8_t> judgeds;
    benough_id_cam.resize(num_cam, false);
    for (; iter_1 != mspKeyFrames.begin(); iter_1 = iter--) {
      KeyFrame *pKF = *iter;
      int8_t id_cam = pKF->id_cam_;
      assert(id_cam < num_cam);
      if (judgeds.end() == judgeds.find(id_cam)) {
        judgeds.emplace(id_cam);
        time_end[id_cam] = pKF->timestamp_;
      } else if (judgeds.size() == num_cam)
        break;
    }
    judgeds.clear();
    for (iter = mspKeyFrames.begin(); iter != mspKeyFrames.end(); ++iter) {
      KeyFrame *pKF = *iter;
      int8_t id_cam = pKF->id_cam_;
      assert(id_cam < num_cam);
      if (judgeds.end() == judgeds.find(id_cam)) {
        judgeds.emplace(id_cam);
        if (time_end[id_cam] - pKF->timestamp_ >= time_span) {
          benough_id_cam[id_cam] = true;
        }
      } else if (judgeds.size() == num_cam)
        break;
    }
    return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
  }

  vector<KeyFrame *> vKFs;
  vector<double> time_last = time_end;
  vector<bool> bfinished(num_cam, false);
  vector<size_t> num_kf(num_cam, 0);
  int enough_cam_num = 0, finished_cam_num = 0;
  benough_id_cam.clear();
  benough_id_cam.resize(num_cam, false);
  for (; iter_1 != mspKeyFrames.begin(); iter_1 = iter--) {
    KeyFrame *pKF = *iter;
    if ((!fix_mode && pKF->bcam_fixed_) || (1 == fix_mode && !pKF->bcam_fixed_)) continue;
    double time_ref = pKF->timestamp_;
    int8_t id_cam = pKF->id_cam_;
    assert(id_cam < num_cam);
    if (time_end[id_cam] < time_ref) {
      time_end[id_cam] = time_ref;
      time_last[id_cam] = time_ref;
    }
    if (!bfinished[id_cam]) {
      if (time_end[id_cam] - time_ref <= time_span_max && time_last[id_cam] - time_ref <= max_delta_tij) {
        vKFs.insert(vKFs.begin(), *iter);
        ++num_kf[id_cam];

        if (time_end[id_cam] - time_ref >= time_span && num_kf[id_cam] >= min_num) {
          bfinished[id_cam] = true;
          ++finished_cam_num;

          ++enough_cam_num;
          benough_id_cam[id_cam] = true;
        }
      } else {
        bfinished[id_cam] = true;
        ++finished_cam_num;
      }
    }
    if (enough_cam_num == num_cam || finished_cam_num == num_cam) break;

    time_last[id_cam] = time_ref;
  }
  return vKFs;
}

void Map::InformNewBigChange() {
  unique_lock<mutex> lock(mMutexMap);
  ++mnBigChangeIdx;
  ++mnChangeIdx;  // zzh
}

int Map::GetLastBigChangeIdx() {
  unique_lock<mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

vector<KeyFrame *> Map::GetAllKeyFrames() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<MapPoint *> Map::GetAllMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

vector<MapPoint *> Map::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid() {
  unique_lock<mutex> lock(mMutexMap);
  return mnMaxKFid;
}

void Map::clear() {
  for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++) delete *sit;

  for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++) delete *sit;

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = 0;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

}  // namespace VIEO_SLAM
