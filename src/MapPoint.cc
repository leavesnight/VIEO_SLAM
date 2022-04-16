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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "KannalaBrandt8.h"
#include "common/log.h"

#include <mutex>

namespace VIEO_SLAM {

void MapPoint::UpdateScale(const float& scale) {
  unique_lock<mutex> lock(mMutexPos);
  mfMaxDistance *= scale;
  mfMinDistance *= scale;
  SetWorldPos(mWorldPos * scale, false);
}

// for Load/SaveMap()
MapPoint::MapPoint(KeyFrame* pRefKF, Map* pMap, istream& is)
    : mnFirstKFid(pRefKF->mnId),
      nObs(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),  // mnVisible&mnFound will be set by TrackLocalMap() in Tracking.cc, used in LocalMapping.cc
      mpReplaced(static_cast<MapPoint*>(NULL)),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap) {
  read(is);
  mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}
bool MapPoint::read(istream& is) {
  // we read mnId (old id),refKF's old id in LoadMap()
  {
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos.create(3, 1, CV_32F);                     // allocate if needed
    is.read((char*)mWorldPos.data, sizeof(float) * 3);  // float xyz
  }
  return is.good();
}
bool MapPoint::write(ostream& os) {
  // we save mnId,refKF's old id in SaveMap()
  os.write((char*)(GetWorldPos().data), sizeof(float) * 3);  // float xyz
  return os.good();
}

// added by zzh

long unsigned int MapPoint::nNextId = 0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap)
    : mnFirstKFid(pRefKF->mnId),
      nObs(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(static_cast<MapPoint*>(NULL)),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap) {
  Pos.copyTo(mWorldPos);
  mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

MapPoint::MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame, const int& idxF)
    : mnFirstKFid(-1),
      nObs(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(static_cast<KeyFrame*>(NULL)),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(NULL),
      mpMap(pMap) {
  Pos.copyTo(mWorldPos);
  // similar to part in the StereoInitialization(): Update Normal&Depth Compute Descriptor
  //  TODO(zzh): check dist&level from multicams instead of current ref cam 0
  cv::Mat Ow = pFrame->GetCameraCenter();
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector / cv::norm(mNormalVector);  // normalized normal vector

  cv::Mat PC = Pos - Ow;
  const float dist = cv::norm(PC);
  const int level = pFrame->mvKeys[idxF].octave;  // Un
  const float levelScaleFactor = pFrame->mvScaleFactors[level];
  const int nLevels = pFrame->mnScaleLevels;

  mfMaxDistance = dist * levelScaleFactor;
  mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat& Pos, bool block) {
  unique_lock<mutex> lock2(mGlobalMutex);
  unique_lock<mutex> lock(mMutexPos, defer_lock);
  if (block) lock.lock();
  Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos() {
  unique_lock<mutex> lock(mMutexPos);
  return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal() {
  unique_lock<mutex> lock(mMutexPos);
  return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  // for EraseObs in different threads may cause some thread addobs happens on bad mp
  if (mbBad) return;

  set<size_t> indexes;
  auto iter = mObservations.find(pKF);
  if (mObservations.end() != iter) {
    indexes = iter->second;
  }
  CV_Assert(indexes.end() == indexes.find(idx));
  indexes.insert(idx);
  mObservations[pKF] = indexes;
  PRINT_DEBUG_INFO_MUTEX(mnId << "add obs" << idx << " ", imu_tightly_debug_path, "debug.txt");

  if (pKF->mvuRight[idx] >= 0)
    nObs += 2;
  else
    nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF, size_t idx) {
  bool bBad = false;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    auto iterobs = mObservations.find(pKF);
    if (mObservations.end() != iterobs) {
      auto& idxs = iterobs->second;
      if (-1 == idx) {
        for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
          auto idx = *iter;
          if (pKF->mvuRight[idx] >= 0)
            nObs -= 2;
          else
            nObs--;
        }

        mObservations.erase(pKF);
      } else {
        if (pKF->mvuRight[idx] >= 0)
          nObs -= 2;
        else
          --nObs;
        CV_Assert(idxs.count(idx) && "Error for empty idx erased!");
        idxs.erase(idx);

        if (idxs.empty()) mObservations.erase(pKF);
      }

      // If < 2 observing KFs for a stereo MP, discard it for it may be created by triangulation method!
      if (nObs <= 2) bBad = true;
      //! empty()/nObs>0 avoids for Segmentation Fault? revised by zzh, notice we don't use the information of bad MPs
      else if (mpRefKF == pKF) {
        mpRefKF = mObservations.begin()->first;
      }
    }
  }

  if (bBad) SetBadFlag();
}

map<KeyFrame*, set<size_t>> MapPoint::GetObservations() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mObservations;
}

int MapPoint::Observations() {
  unique_lock<mutex> lock(mMutexFeatures);
  return nObs;
}

void MapPoint::SetBadFlag() {
  map<KeyFrame*, set<size_t>> obs;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    mbBad = true;
    obs = mObservations;
    mObservations.clear();
  }
  for (map<KeyFrame*, set<size_t>>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
    KeyFrame* pKF = mit->first;
    auto idxs = mit->second;
    for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
      auto idx = *iter;
      pKF->EraseMapPointMatch(idx);
    }
  }

  mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced() {
  unique_lock<mutex> lock1(mMutexFeatures);
  unique_lock<mutex> lock2(mMutexPos);
  return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP) {
  if (pMP->mnId == this->mnId) return;

  PRINT_DEBUG_INFO_MUTEX(pMP->mnId << "replace" << mnId << " ", imu_tightly_debug_path, "debug.txt");
  int nvisible, nfound;
  map<KeyFrame*, set<size_t>> obs;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    // for lba may eraseobs which may setbad mp then gba may replace bad mp, causing mpReplaced->mpReplaced to form a
    // dead cycle
    if (pMP->isBad()) return;
    obs = mObservations;
    mObservations.clear();
    mbBad = true;
    nvisible = mnVisible;
    nfound = mnFound;
    mpReplaced = pMP;
  }

  for (map<KeyFrame*, set<size_t>>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
    // Replace measurement in keyframe
    KeyFrame* pKF = mit->first;

    auto idxs = mit->second;
    for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
      auto idx = *iter;
      size_t cami = pKF->mapn2in_.size() <= idx ? 0 : get<0>(pKF->mapn2in_[idx]);
      if (!pMP->IsInKeyFrame(pKF, -1, cami)) {
        pKF->ReplaceMapPointMatch(idx, pMP);
        pMP->AddObservation(pKF, idx);
      } else {
        // just erase pKF->mvpMapPoints[mit->second] for it already exists/matches in another
        // pKF->mvpMapPoints[idx](idx!=mit->second)
        auto idxs_old = pMP->GetObservations()[pKF];
        CV_Assert(idxs_old.end() == idxs_old.find(idx));
        pKF->EraseMapPointMatch(idx);
        PRINT_DEBUG_INFO_MUTEX("erase:" << pKF->mnId << "," << idx << endl, imu_tightly_debug_path, "debug.txt");
      }
    }
  }
  pMP->IncreaseFound(nfound);
  pMP->IncreaseVisible(nvisible);
  pMP->ComputeDistinctiveDescriptors();  // why don't calculate the normal?

  mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad() {
  unique_lock<mutex> lock(mMutexFeatures);
  unique_lock<mutex> lock2(mMutexPos);
  return mbBad;
}

void MapPoint::IncreaseVisible(int n) {
  unique_lock<mutex> lock(mMutexFeatures);
  mnVisible += n;
}

void MapPoint::IncreaseFound(int n) {
  unique_lock<mutex> lock(mMutexFeatures);
  mnFound += n;
}

float MapPoint::GetFoundRatio() {
  unique_lock<mutex> lock(mMutexFeatures);
  return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  vector<cv::Mat> vDescriptors;

  map<KeyFrame*, set<size_t>> observations;

  {
    unique_lock<mutex> lock1(mMutexFeatures);
    if (mbBad) return;
    observations = mObservations;
  }

  if (observations.empty()) return;

  vDescriptors.reserve(observations.size());

  for (map<KeyFrame*, set<size_t>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend;
       mit++) {
    KeyFrame* pKF = mit->first;

    // TODO: check if this needs to be extended for 4 cams
    if (!pKF->isBad()) {
      auto idxs = mit->second;
      for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
        auto idx = *iter;
        vDescriptors.push_back(pKF->mDescriptors.row(idx));
      }
    }
  }

  if (vDescriptors.empty()) return;

  // Compute distances between them
  const size_t N = vDescriptors.size();

  float Distances[N][N];
  for (size_t i = 0; i < N; i++) {
    Distances[i][i] = 0;
    for (size_t j = i + 1; j < N; j++) {
      int distij = ORBmatcher::DescriptorDistance(
          vDescriptors[i], vDescriptors[j]);  // the hamming distance of the 256 bit descriptor(at the fastest way)
      Distances[i][j] = distij;
      Distances[j][i] = distij;
    }
  }

  // Take the descriptor with least median distance to the rest
  int BestMedian = INT_MAX;
  int BestIdx = 0;
  for (size_t i = 0; i < N; i++) {
    vector<int> vDists(Distances[i], Distances[i] + N);
    sort(vDists.begin(), vDists.end());
    int median = vDists[0.5 * (N - 1)];

    if (median < BestMedian) {
      BestMedian = median;
      BestIdx = i;
    }
  }

  {
    unique_lock<mutex> lock(mMutexFeatures);
    mDescriptor = vDescriptors[BestIdx].clone();
  }
}

cv::Mat MapPoint::GetDescriptor() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mDescriptor.clone();
}

set<size_t> MapPoint::GetIndexInKeyFrame(KeyFrame* pKF) {
  unique_lock<mutex> lock(mMutexFeatures);
  if (mObservations.count(pKF))
    return mObservations[pKF];
  else
    return set<size_t>();
}

bool MapPoint::IsInKeyFrame(KeyFrame* pKF, size_t idx, size_t cami) {
  unique_lock<mutex> lock(mMutexFeatures);
  auto iter = mObservations.find(pKF);
  if (mObservations.end() == iter) return false;
  if (-1 == idx) {
    if (-1 == cami)
      return true;
    else {
      if (!pKF->mapn2in_.size()) return 0 == cami;
      bool ret = false;
      for (auto iteridx : iter->second) {
        if (cami == get<0>(pKF->mapn2in_[iteridx])) {
          ret = true;
          break;
        }
      }
      return ret;
    }
  } else if (iter->second.end() == iter->second.find(idx))
    return false;
  else if (-1 == cami)
    return true;
  else {
    if (pKF->mapn2in_.size() > idx) {
      return cami == get<0>(pKF->mapn2in_[idx]);
    } else
      return 0 == cami;
  }
}

void MapPoint::UpdateNormalAndDepth() {
  map<KeyFrame*, set<size_t>> observations;
  KeyFrame* pRefKF;
  cv::Mat Pos;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    if (mbBad) return;
    observations = mObservations;
    pRefKF = mpRefKF;
    Pos = mWorldPos.clone();
  }

  if (observations.empty()) return;

  cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
  int n = 0;
  for (map<KeyFrame*, set<size_t>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend;
       mit++) {
    KeyFrame* pKF = mit->first;
    cv::Mat twcr = pKF->GetCameraCenter();
    cv::Mat twc = twcr.clone();
    auto idxs = mit->second;
    for (auto iter = idxs.begin(), iterend = idxs.end(); iter != iterend; ++iter) {
      auto idx = *iter;
      size_t cami = pKF->mapn2in_.size() <= idx ? 0 : get<0>(pKF->mapn2in_[idx]);
      if (pKF->mpCameras.size() > cami) {
        GeometricCamera* pcam1 = pKF->mpCameras[cami];
        const cv::Mat Rcrw = pKF->GetRotation();
        twc += Rcrw.t() * pcam1->Getcvtrc();
      }
      cv::Mat normali = Pos - twc;
      normal = normal + normali / cv::norm(normali);
      n++;
    }
  }

  // TODO(zzh): check dist&level from multicams instead of current ref cam 0
  cv::Mat PC = Pos - pRefKF->GetCameraCenter();
  const float dist =
      cv::norm(PC);  // why not dist*cos(theta)? then we have deltaPatch'/deltaPatch=dist/dist'(without rotation)
  auto& trackinfo = GetTrackInfoRef();
  if (INFINITY == trackinfo.track_depth_) trackinfo.track_depth_ = dist;
  CV_Assert(observations.find(pRefKF) != observations.end());
  const int level = pRefKF->mvKeys[*observations[pRefKF].begin()].octave;  // Un
  const float levelScaleFactor = pRefKF->mvScaleFactors[level];
  const int nLevels = pRefKF->mnScaleLevels;

  {
    unique_lock<mutex> lock3(mMutexPos);
    mfMaxDistance = dist * levelScaleFactor;                              // dist*1.2^level
    mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];  // fMaxDis/1.2^7
    mNormalVector = normal / n;  // here maybe use normal/cv::norm(normal) better?
  }
}

float MapPoint::GetMinDistanceInvariance() {
  unique_lock<mutex> lock(mMutexPos);
  return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance() {
  unique_lock<mutex> lock(mMutexPos);
  return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float& currentDist, KeyFrame* pKF) {
  float ratio;
  {
    unique_lock<mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pKF->mnScaleLevels)
    nScale = pKF->mnScaleLevels - 1;

  return nScale;
}

int MapPoint::PredictScale(const float& currentDist, Frame* pF) {
  float ratio;
  {
    unique_lock<mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);  // log(CurDist*1.2^level/CurDist)/log(1.2)=level, notice here
                                                         // use ceil, so nScale-1 is also ok
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pF->mnScaleLevels)
    nScale = pF->mnScaleLevels - 1;

  return nScale;
}

void MapPoint::_TrackFastMatchInfo::Reset(Frame* pf) {
  //  if (!pf) {
  btrack_inview_ = false;
  for (int i = 0; i < NUM_PROJ; ++i) {
    vtrack_proj_[i].clear();
  }
  vtrack_scalelevel_.clear();
  vtrack_viewcos_.clear();
  vtrack_cami_.clear();
  //  return;
  //  }

  if (pf) last_seen_frameid_ = pf->mnId;  // don't need to match it in SBP()

  //  if (vtrack_cami_.empty()) return;
  //  auto cami = pf->mapn2in_.size() <= i ? 0 : get<0>(pf->mapn2in_[i]);
  //  //  bool berase = false;
  //  auto iter_scale_level = vtrack_scalelevel_.begin();
  //  auto iter_viewcos = vtrack_viewcos_.begin();
  //  list<float>::iterator iter_proj[NUM_PROJ];
  //  for (int k = 0; k < NUM_PROJ; ++k) iter_proj[k] = vtrack_proj_[k].begin();
  //  auto IncIter = [&]() {
  //    for (int k = 0; k < NUM_PROJ; ++k) ++iter_proj[k];
  //    ++iter_scale_level;
  //    ++iter_viewcos;
  //  };
  //  for (auto iter_cami = vtrack_cami_.begin(); iter_cami != vtrack_cami_.end();) {
  //    if (cami != *iter_cami) {
  //      ++iter_cami;
  //      IncIter();
  //      continue;
  //    }
  //    iter_cami = vtrack_cami_.erase(iter_cami);
  //    for (int k = 0; k < NUM_PROJ; ++k) iter_proj[k] = vtrack_proj_[k].erase(iter_proj[k]);
  //    iter_scale_level = vtrack_scalelevel_.erase(iter_scale_level);
  //    iter_viewcos = vtrack_viewcos_.erase(iter_viewcos);
  //    //    berase = true;
  //    break;
  //  }
}

}  // namespace VIEO_SLAM
