//
// Created by leavesnight on 2021/12/20.
//

#pragma once

#include <stddef.h>
#include <vector>
#include <array>
#include <set>
#include "GeometricCamera.h"
#include "Converter.h"
#include "common/so3_extra.h"
#include "NavState.h"
#include <typeinfo>
#include "OdomPreIntegrator.h"
#include "common/type_def.h"
#include "loop/DBoW2/DBoW2/BowVector.h"
#include "loop/DBoW2/DBoW2/FeatureVector.h"
#include "loop/DBoW2/DBoW2/FORB.h"

namespace DBoW2 {
template <class TDescriptor, class F>
/// Generic Vocabulary
class TemplatedVocabulary;
}

namespace VIEO_SLAM {
class MapPoint;
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
using common::TS2S;

class FrameBase {
 public:  // OdomFrameBase related
  template <typename _Tp>
  using vector = std::vector<_Tp>;
  template <typename _Tp, std::size_t _Nm>
  using array = std::array<_Tp, _Nm>;
  template <typename _Key>
  using set = std::set<_Key>;
  template <typename _Key, typename _Tp>
  using map = std::map<_Key, _Tp>;
  template <typename _T1, typename _T2>
  using pair = std::pair<_T1, _T2>;
  using ostream = std::ostream;
  using istream = std::istream;

  FrameBase() {}
  FrameBase(const double &timestamp) : timestamp_(timestamp), ftimestamp_(TS2S(timestamp)) {}
  virtual ~FrameBase() {}

  // TODO: if Trc ref is imu, here need to be changed
  virtual const Sophus::SE3d GetTwc();
  virtual const Sophus::SE3d GetTcw();
  Sophus::SE3d GetTcr() { return Sophus::SE3d(); }

  virtual void AddMapPoint(MapPoint *pMP, const size_t &idx);
  virtual void EraseMapPointMatch(const size_t &idx) { mvpMapPoints[idx] = nullptr; }
  virtual vector<MapPoint *> GetMapPointMatches() { return mvpMapPoints; }
  virtual void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) { mvpMapPoints[idx] = pMP; }
  virtual set<pair<MapPoint *, size_t>> GetMapPointsCami();

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

  // Frame timestamp.
  double timestamp_;  // TODO(zzh): change to TimeStampNs
  double ftimestamp_;
  int8_t id_cam_ = 0;
  int8_t bcam_fixed_ = 1;

  // const Tbc,Tce, so it can be used in multi threads
  static cv::Mat mTbc, mTce;
  static Eigen::Matrix3d meigRcb;
  static Eigen::Vector3d meigtcb;
  static bool usedistort_;
  static bool busedist_set_;
  vector<GeometricCamera *> mpCameras;

 public:  // BAFrameBase related
  virtual bool isBad() { return mbBad; }

  // Number of KeyPoints. The left members are all associated by the index
  int N;
  // Vector of original keypoints and undistorted kpts
  vector<cv::KeyPoint> mvKeys;
  vector<cv::KeyPoint> mvKeysUn;
  vector<pair<size_t, size_t>> mapn2in_;
  // the left associated members are from mDescriptors

  long unsigned int mnId;

  // flow related
  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

 public:  // FrameBase related
  // Compute Bag of Words representation.
  void ComputeBoW();  // compute mBowVec && mFeatVec

  // KeyPoint functions
  // return vec<featureID>, a 2r*2r window search by Grids/Cells speed-up, min/maxlevel check is for Frame
  vector<size_t> GetFeaturesInArea(uint8_t cami, const float &x, const float &y, const float &r,
                                   const int minlevel = -1, const int maxlevel = -1) const;
  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
  void AssignFeaturesToGrid();
  // Compute the cell of a keypoint (return false if outside the grid)
  bool PosInGrid(uint8_t cami, const cv::KeyPoint &kp, int &posX, int &posY);
  // TODO: move these to cam model
  bool IsInImage(uint8_t cami, const float &x, const float &y) const;
  // Computes image bounds for the (un)distorted image (called in the constructor).
  void ComputeImageBounds(const vector<int> &wid_hei);

  virtual bool read(istream &is);
  virtual bool write(ostream &os) const;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors;
  // Vocabulary used for relocalization.
  ORBVocabulary *mpORBvocabulary;
  // BoW: Bag of Words Vector structures.
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;
  // for keyframe judge and culling, filled in frame (half)constructor, unchanged after
  typedef struct _StereoInfo {
    // Corresponding stereo depth and right coordinate for each keypoint.
    vector<float> vdepth_;  // this size(n) >= v3dpoints_.size()(idxs)
    // to speed up ba for (un)distorted RGBD/(rectified) stereo; <0 means "Monocular" keypoints
    vector<float> vuright_;
    // Triangulated stereo observations in reference frame. for ComputeStereoXXX() and UnprojectStereo()
    aligned_vector<Vector3d> v3dpoints_;                // keep same size with vidxs_matches
    vector<bool> goodmatches_;                          // keep same size with vidxs_matches
    map<pair<size_t, size_t>, size_t> mapcamidx2idxs_;  // final size_t max < mvidxsMatches.size()
    // Stereo baseline in meters; bf means Stereo baseline multiplied by fx; bf = b * f
    float baseline_bf_[2] = {15.f / 250, 15.f};
  } StereoInfo;
  // mean if this key pt or mp is triangulated by precalibrated Tcicj
  StereoInfo stereoinfo_;
  size_t GetMapn2idxs(size_t i);

  // some unchanged members after constructor func.
  // members inited in derived Frame constructor
  typedef struct _ScalePyramidInfo {
    vector<float> vscalefactor_;     // for fast match radius expansion and CreateNewMP outlier judge
    float fscalefactor_;             // for CreateNewMP outlier judge used threshold
    float flogscalefactor_;          // for fast match radius expansion used predict scale level
    vector<float> vlevelsigma2_;     // for chi2 threshold
    vector<float> vinvlevelsigma2_;  // for BA used Info
  } ScalePyramidInfo;
  // not static for different frame config(like init mono frame or the other)
  // not vec fscalefactors for speeding up and use less space for the same orb extractor params for all cams
  ScalePyramidInfo scalepyrinfo_;

 protected:  // OdomFrameBase related
  inline const Sophus::SE3d GetTcwCst() const {
    auto Tcw = Sophus::SE3d(Sophus::SO3exd(Converter::toMatrix3d(Tcw_.rowRange(0, 3).colRange(0, 3))),
                            Converter::toVector3d(Tcw_.col(3)));
    return Tcw;
  }

  // Camera pose.
  cv::Mat Tcw_;
  // state xi={Ri,pi,vi,bi}, this xi doesn't include landmarks' state li/mi but include the camera's state xci(for Tbc
  // is a constant) not designed for multi threads/just used in Tracking thread in Base
  NavState mNavState;

  // Odom PreIntegration, j means this fb, i means last fb, if no measurements=>mdeltatij==0
  EncPreIntegrator mOdomPreIntEnc;
  IMUPreintegrator mOdomPreIntIMU;

 protected:  // BAFrameBase related
  // Bad flags
  bool mbBad = false;

  // MapPoints associated to keypoints(same order), nullptr if no association.
  vector<MapPoint *> mvpMapPoints;

 protected:  // FrameBase related
  // for GetFeaturesInArea: Grid over the image to speed up feature matching
  // static for speeding up and use less space for the same camera models for all fbs
  typedef struct _GridInfo {
    vector<float> fgrids_widthinv_;
    vector<float> fgrids_heightinv_;
    const int FRAME_GRID_ROWS = 48;
    const int FRAME_GRID_COLS = 64;
    // (Un)distorted Image Bounds (computed once).
    vector<array<float, 4>> minmax_xy_;  // minx,maxx,miny,maxy
    using Tsize = int;
    Tsize sz_dims_[2];  // 0 width,1 height
  } GridInfo;
  static GridInfo gridinfo_;
  // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
  vector<vector<vector<size_t>>> vgrids_;  //[cami][x*ROWS+y][id_kp]
};

// speicalize
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
