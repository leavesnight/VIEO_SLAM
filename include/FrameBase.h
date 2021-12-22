//
// Created by leavesnight on 2021/12/20.
//

#ifndef VIEO_SLAM_FRAMEBASE_H
#define VIEO_SLAM_FRAMEBASE_H

#include <stddef.h>
#include <vector>
#include <set>
#include "GeometricCamera.h"

namespace VIEO_SLAM {
class MapPoint;

class FrameBase {
 public:
  FrameBase() {}
  virtual ~FrameBase() {}
  virtual void AddMapPoint(MapPoint *pMP, const size_t &idx);
  virtual void EraseMapPointMatch(const size_t &idx) { mvpMapPoints[idx] = nullptr; }
  virtual const std::vector<MapPoint *> &GetMapPointMatches() { return mvpMapPoints; }
  const std::vector<MapPoint *> &GetMapPointMatches() const { return mvpMapPoints; }
  virtual void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP) { mvpMapPoints[idx] = pMP; }
  virtual std::set<MapPoint*> GetMapPoints();
  virtual std::set<std::pair<MapPoint*, size_t>> GetMapPointsCami();

  std::vector<GeometricCamera*> mpCameras;
  std::vector<std::pair<size_t, size_t>> mapn2in_;

 protected:
  // MapPoints associated to keypoints(same order), NULL pointer if no association.
  std::vector<MapPoint *> mvpMapPoints;
};
}  // namespace VIEO_SLAM

#endif  // VIEO_SLAM_FRAMEBASE_H
