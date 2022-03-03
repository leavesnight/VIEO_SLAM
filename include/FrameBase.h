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
#include "NavState.h"
#include <typeinfo>
#include "OdomPreIntegrator.h"

namespace VIEO_SLAM {
class MapPoint;

class FrameBase {
 public:
  FrameBase() {}
  FrameBase(const double &timestamp) : mTimeStamp(timestamp) {}
  virtual ~FrameBase() {}

  // TODO: if Trc ref is imu, here need to be changed
  virtual const Sophus::SE3d GetTwc();
  virtual const Sophus::SE3d GetTcw();
  Sophus::SE3d GetTcr() { return Sophus::SE3d(); }

  virtual void AddMapPoint(MapPoint *pMP, const size_t &idx);
  virtual void EraseMapPointMatch(const size_t &idx) { mvpMapPoints[idx] = nullptr; }
  virtual std::vector<MapPoint *> GetMapPointMatches() { return mvpMapPoints; }
  virtual void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) { mvpMapPoints[idx] = pMP; }
  virtual std::set<MapPoint *> GetMapPoints();
  virtual std::set<std::pair<MapPoint *, size_t>> GetMapPointsCami();

  // virtual makes it could be implemented as thread-safe one and used by PreIntegration()
  virtual NavState GetNavState(void) {  // cannot use const &(make mutex useless)
    return mNavState;                   // call copy constructor
  }
  virtual void SetNavState(const NavState &ns) { mNavState = ns; }
  // for SetBadFlag()(just for safety, usually check mpPrevKF is enough)
  virtual EncPreIntegrator GetEncPreInt(void) {
    return mOdomPreIntEnc;  // won't copy list
  }
  virtual IMUPreintegrator GetIMUPreInt(void) {
    return mOdomPreIntIMU;  // won't copy list
  }
  // Notice that here for virtual cannot be applied on template func., be careful for kf's preint op.
  template <class OdomData>
  void ClearOdomPreInt() {
    mOdomPreIntEnc.mdeltatij = 0;
  }
  //[iteri,iterj) IMU preintegration, breset=false could make KF2KF preintegration time averaged to per frame &&
  // connect 2KFs preintegration by only preintegrating the final KF2KF period
  template <class OdomData, class Preintegrator>
  int PreIntegration(typename OdomData::TTtime tm_start, typename OdomData::TTtime tm_end,
                     const Eigen::Vector3d &bgi_bar, const Eigen::Vector3d &bai_bar,
                     const typename aligned_list<OdomData>::const_iterator &iteri,
                     const typename aligned_list<OdomData>::const_iterator &iterj, bool breset = true,
                     Preintegrator *ppreint_odom = nullptr, int8_t verbose = 0) {
    if (!ppreint_odom) ppreint_odom = &mOdomPreIntEnc;
    return ppreint_odom->PreIntegration(tm_start, tm_end, iteri, iterj, breset);
  }  // 0th frame don't use this function, pLastF shouldn't be bad
  template <class OdomData>
  void PreIntegration(FrameBase *plastfb, const typename aligned_list<OdomData>::const_iterator &iteri,
                      const typename aligned_list<OdomData>::const_iterator &iterj, bool breset = true,
                      int8_t verbose = 0) {
    NavState ns = plastfb->GetNavState();  // unused for EncData, but specialized and used for IMUData
    PreIntegration<OdomData, EncPreIntegrator>(plastfb->mTimeStamp, mTimeStamp, ns.mbg, ns.mba, iteri, iterj, breset,
                                               nullptr, verbose);
  }

  std::vector<GeometricCamera *> mpCameras;
  std::vector<std::pair<size_t, size_t>> mapn2in_;

  // Frame timestamp.
  double mTimeStamp;

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

  // state xi={Ri,pi,vi,bi}, this xi doesn't include landmarks' state li/mi but include the camera's state xci(for Tbc
  // is a constant) not designed for multi threads/just used in Tracking thread in Base
  NavState mNavState;
  // Odom PreIntegration, j means this fb, i means last fb, if no measurements=>mdeltatij==0
  EncPreIntegrator mOdomPreIntEnc;
  IMUPreintegrator mOdomPreIntIMU;
};

template <>
void FrameBase::ClearOdomPreInt<IMUData>();
template <>
int FrameBase::PreIntegration<IMUData, IMUPreintegrator>(IMUData::TTtime tm_start, IMUData::TTtime tm_end,
                                                         const Eigen::Vector3d &bgi_bar, const Eigen::Vector3d &bai_bar,
                                                         const typename aligned_list<IMUData>::const_iterator &iteri,
                                                         const typename aligned_list<IMUData>::const_iterator &iterj,
                                                         bool breset, IMUPreintegrator *ppreint_odom, int8_t verbose);
template <>
void FrameBase::PreIntegration<IMUData>(FrameBase *plastfb, const typename aligned_list<IMUData>::const_iterator &iteri,
                                        const typename aligned_list<IMUData>::const_iterator &iterj, bool breset,
                                        int8_t verbose);
}  // namespace VIEO_SLAM

#endif  // VIEO_SLAM_FRAMEBASE_H
