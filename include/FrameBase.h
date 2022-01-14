//
// Created by leavesnight on 2021/12/20.
//

#ifndef VIEO_SLAM_FRAMEBASE_H
#define VIEO_SLAM_FRAMEBASE_H

#include <stddef.h>
#include <vector>
#include <set>
#include "GeometricCamera.h"
#include "Converter.h"
#include "so3_extra.h"

namespace VIEO_SLAM {
class MapPoint;

class FrameBase {
 public:
  FrameBase() {}
  virtual ~FrameBase() {}

  // TODO: if Trc ref is imu, here need to be changed
  virtual const Sophus::SE3d GetTwc();
  virtual const Sophus::SE3d GetTcw();
  Sophus::SE3d GetTcr() { return Sophus::SE3d(); }

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
  inline const Sophus::SE3d GetTcwCst() const {
    auto Tcw = Sophus::SE3d(Sophus::SO3exd(Converter::toMatrix3d(Tcw_.rowRange(0, 3).colRange(0, 3))),
                            Converter::toVector3d(Tcw_.col(3)));
    return Tcw;
  }

  // MapPoints associated to keypoints(same order), NULL pointer if no association.
  std::vector<MapPoint *> mvpMapPoints;

  // Camera pose.
  cv::Mat Tcw_;
};
}  // namespace VIEO_SLAM

#endif  // VIEO_SLAM_FRAMEBASE_H
