//
// Created by leavesnight on 2021/12/20.
//

#include "FrameBase.h"
#include "MapPoint.h"

using namespace VIEO_SLAM;
using std::pair;
using std::set;

const Sophus::SE3d FrameBase::GetTwc() { return GetTcw().inverse(); }
const Sophus::SE3d FrameBase::GetTcw() { return GetTcwCst(); }

void FrameBase::AddMapPoint(MapPoint *pMP, const size_t &idx) {
  assert(mvpMapPoints.size() > idx);
  //  if (mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad()) {
  //    cout << "check mpid=" << mvpMapPoints[idx]->mnId << " ";
  //    auto vobs = mvpMapPoints[idx]->GetObservations();
  //    for (auto obs : vobs) {
  //      cout << obs.first->mnId << ":";
  //      for (auto idx : obs.second) cout << idx << " ";
  //      cout << endl;
  //    }
  //    cout << endl;
  //  }
  assert(!mvpMapPoints[idx] || mvpMapPoints[idx]->isBad());
  mvpMapPoints[idx] = pMP;
}

std::set<std::pair<MapPoint *, size_t>> FrameBase::GetMapPointsCami() {
  set<pair<MapPoint *, size_t>> s;
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i]) continue;
    MapPoint *pMP = mvpMapPoints[i];
    if (!pMP->isBad()) {
      size_t cami = mapn2in_.size() <= i ? 0 : get<0>(mapn2in_[i]);
      s.insert(make_pair(pMP, cami));
    }
  }
  return s;
}

template <>
void FrameBase::ClearOdomPreInt<IMUData>() {
  mOdomPreIntIMU.mdeltatij = 0;
}
template <>
int FrameBase::PreIntegration<IMUData, IMUPreintegrator>(IMUData::TTtime tm_start, IMUData::TTtime tm_end,
                                                         const Eigen::Vector3d &bgi_bar, const Eigen::Vector3d &bai_bar,
                                                         const typename aligned_list<IMUData>::const_iterator &iteri,
                                                         const typename aligned_list<IMUData>::const_iterator &iterj,
                                                         bool breset, IMUPreintegrator *ppreint_odom, int8_t verbose) {
  if (!ppreint_odom) ppreint_odom = &mOdomPreIntIMU;
#ifndef TRACK_WITH_IMU
  // TODO: fix this
  return ppreint_odom->PreIntegration(tm_start, tm_end, bgi_bar, bai_bar, iteri, iterj, breset);
#else
  return ppreint_odom->PreIntegration(tm_start, tm_end, bgi_bar, bai_bar, iteri, iterj, breset);
#endif
}
template <>
void FrameBase::PreIntegration<IMUData>(FrameBase *plastfb, const typename aligned_list<IMUData>::const_iterator &iteri,
                                        const typename aligned_list<IMUData>::const_iterator &iterj, bool breset,
                                        int8_t verbose) {
  NavState ns = plastfb->GetNavState();
  PreIntegration<IMUData, IMUPreintegrator>(plastfb->mTimeStamp, mTimeStamp, ns.mbg, ns.mba, iteri, iterj, breset,
                                            nullptr, verbose);
}
