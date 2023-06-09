//
// Created by leavesnight on 2021/12/20.
//

#pragma once

#include <stddef.h>
#include <vector>
#include <set>
#include "GeometricCamera.h"
#include "Converter.h"
#include "common/so3_extra.h"
#include "NavState.h"
#include <typeinfo>
#include "OdomPreIntegrator.h"
#include "common/type_def.h"

namespace VIEO_SLAM {
class MapPoint;
using common::TS2S;

class FrameBase {
 public:
  template <typename _Tp>
  using vector = std::vector<_Tp>;
  using ostream = std::ostream;
  using istream = std::istream;

  // const Tbc,Tce, so it can be used in multi threads
  static cv::Mat mTbc, mTce;
  static Eigen::Matrix3d meigRcb;
  static Eigen::Vector3d meigtcb;

  long unsigned int mnId;

  FrameBase() {}
  FrameBase(const double &timestamp) : timestamp_(timestamp), ftimestamp_(TS2S(timestamp)) {}
  virtual ~FrameBase() {}

  // TODO: if Trc ref is imu, here need to be changed
  virtual const Sophus::SE3d GetTwc();
  virtual const Sophus::SE3d GetTcw();
  Sophus::SE3d GetTcr() { return Sophus::SE3d(); }

  virtual void AddMapPoint(MapPoint *pMP, const size_t &idx);
  virtual void EraseMapPointMatch(const size_t &idx) { mvpMapPoints[idx] = nullptr; }
  virtual std::vector<MapPoint *> GetMapPointMatches() { return mvpMapPoints; }
  virtual void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) { mvpMapPoints[idx] = pMP; }
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
    PreIntegration<OdomData, EncPreIntegrator>(plastfb->ftimestamp_, ftimestamp_, ns.mbg, ns.mba, iteri, iterj, breset,
                                               nullptr, verbose);
  }

  virtual bool isBad() { return mbBad; }

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  std::vector<GeometricCamera *> mpCameras;
  std::vector<std::pair<size_t, size_t>> mapn2in_;

  // Frame timestamp.
  double timestamp_;  // TODO(zzh): change to TimeStampNs
  double ftimestamp_;
  int8_t id_cam_ = 0;
  int8_t bcam_fixed_ = 1;

  // flow related
  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints. The left members are all associated by the index
  int N;
  // Vector of original keypoints and undistorted kpts
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysUn;
  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors;
  // for keyframe judge and culling, filled in frame (half)constructor, unchanged after
  typedef struct _StereoInfo {
    // Corresponding stereo depth and right coordinate for each keypoint.
    vector<float> vdepth_;   // this size(n) >= v3dpoints_.size()(idxs)
    vector<float> vuright_;  // to speed up ba for rectified stereo; "Monocular" keypoints have a negative value.
    // Stereo baseline in meters; bf means Stereo baseline multiplied by fx; bf = b * f
    float baseline_bf_[2] = {15.f / 250, 15.f};
  } StereoInfo;
  // mean if this key pt or mp is triangulated by precalibrated Tcicj
  StereoInfo stereoinfo_;

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

  // Bad flags
  bool mbBad = false;

 public:  // for serialize
  // can also be used for set/list
  template <class T>
  static inline bool writeVec(ostream &os, const T &vec);
  template <class T>
  static inline bool writeVecwrite(std::ostream &os, const T &lis);
  static inline bool writeMat(ostream &os, const cv::Mat &mat);
  // for Eigen::Matrix<_Scalar,_Rows,_Cols>
  template <class T>
  static inline bool writeEigMat(ostream &os, const T &mat);
  template <class T>
  static inline bool readVec(istream &is, T &vec);
  template <class T>
  static inline bool readVecread(std::istream &is, T &lis);
  static inline bool readMat(istream &is, cv::Mat &mat);
  template <class T>
  static inline bool readEigMat(istream &is, T &mat);
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

// hpp
template <class T>
bool FrameBase::writeVec(ostream &os, const T &vec) {
  for (typename T::const_iterator iter = vec.begin(); iter != vec.end(); ++iter) {
    os.write((char *)&(*iter), sizeof(*iter));
  }
  return os.good();
}
template <class T>
bool FrameBase::writeVecwrite(std::ostream &os, const T &lis) {
  for (typename T::const_iterator iter = lis.begin(); iter != lis.end(); ++iter) iter->write(os);
  return os.good();
}
bool FrameBase::writeMat(ostream &os, const cv::Mat &mat) {
  for (int i = 0; i < mat.rows; ++i) {
    os.write((char *)mat.ptr(i), mat.cols * mat.elemSize());
  }
  return os.good();
}
template <class T>
bool FrameBase::writeEigMat(ostream &os, const T &mat) {
  // mat.size()==mat.rows()*mat.cols(), saved
  os.write((char *)mat.data(), mat.size() * sizeof(typename T::Scalar));
  // acquiescently as the column-major order
  return os.good();
}
template <class T>
bool FrameBase::readVec(istream &is, T &vec) {
  for (typename T::iterator iter = vec.begin(); iter != vec.end(); ++iter) {
    is.read((char *)&(*iter), sizeof(*iter));
  }
  return is.good();
}
template <class T>
bool FrameBase::readVecread(std::istream &is, T &lis) {
  for (typename T::iterator iter = lis.begin(); iter != lis.end(); ++iter) iter->read(is);
  return is.good();
}
bool FrameBase::readMat(istream &is, cv::Mat &mat) {
  for (int i = 0; i < mat.rows; ++i) {
    is.read((char *)mat.ptr(i), mat.cols * mat.elemSize());
  }
  return is.good();
}
template <class T>
bool FrameBase::readEigMat(istream &is, T &mat) {
  is.read((char *)mat.data(), mat.size() * sizeof(typename T::Scalar));
  return is.good();
}

}  // namespace VIEO_SLAM
