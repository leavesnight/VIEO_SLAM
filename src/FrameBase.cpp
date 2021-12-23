//
// Created by leavesnight on 2021/12/20.
//

#include "FrameBase.h"
#include "MapPoint.h"

using namespace VIEO_SLAM;
using std::set;
using std::pair;

void FrameBase::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
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
  mvpMapPoints[idx]=pMP;
}

set<MapPoint*> FrameBase::GetMapPoints()
{
  set<MapPoint*> s;
  for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
  {
    if(!mvpMapPoints[i])
      continue;
    MapPoint* pMP = mvpMapPoints[i];
    if(!pMP->isBad())
      s.insert(pMP);
  }
  return s;
}

std::set<std::pair<MapPoint*, size_t>> FrameBase::GetMapPointsCami() {
  set<pair<MapPoint*, size_t>> s;
  for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
    if (!mvpMapPoints[i]) continue;
    MapPoint* pMP = mvpMapPoints[i];
    if (!pMP->isBad()) {
      size_t cami = mapn2in_.size() <= i ? 0 : get<0>(mapn2in_[i]);
      s.insert(make_pair(pMP, cami));
    }
  }
  return s;
}