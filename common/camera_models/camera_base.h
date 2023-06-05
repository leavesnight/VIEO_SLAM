//
// Created by leavesnight on 5/30/23.
//

#pragma once

#include <math.h>
#include <memory>
#include "sophus/se3.hpp"
#include "common/eigen_utils.h"
#include "common/interface.h"
#include "common/so3_extra.h"
#include "common/unordered_hash.h"
#include "common/config.h"
#include "common/mlog/log.h"

namespace VIEO_SLAM {
namespace camm {
class GeometricCameraBase {  // designed for body(imu/ref/...) frame to cam:Tbc
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Tdata = FLT_CAMM;
  using Ptr = std::shared_ptr<GeometricCameraBase>;

  using CamId = uint8_t;
  using SE3data = Sophus::SE3<Tdata>;

  GeometricCameraBase(CamId camera_id, const SE3data &Tbc) : camera_id_(camera_id) {
    SetTbc(Tbc);
    SetTrc(Tbc);
  }
  virtual ~GeometricCameraBase() {}

  const SE3data &GetTbc() const { return Tbc_; }
  const SE3data &GetTcb() const { return Tcb_; }
  void SetTbc(const SE3data &Tbc) {
    Tbc_ = Tbc;
    Tcb_ = Tbc.inverse();
  }
  const SE3data &GetTrc() const { return Trc_; }
  const SE3data &GetTcr() const { return Tcr_; }
  void SetTrc(const SE3data &Trc) {
    Trc_ = Trc;
    Tcr_ = Trc.inverse();
  }

  const CamId &camera_id() const { return camera_id_; }

 protected:
  CamId camera_id_;
  SE3data Tbc_;
  // for efficiency
  SE3data Tcb_;
  SE3data Trc_, Tcr_;
};

class CameraBase {
 public:
  using Tdata = FLT_CAMM;
  using Ptr = std::shared_ptr<CameraBase>;

  template <typename _Tp>
  using vector = std::vector<_Tp>;

  using Tsize = int;
  using Tio = double;
  using string = std::string;
  using Vec2data = Eigen::Matrix<Tdata, 2, 1>;
  using Vec3io = Eigen::Matrix<Tio, 3, 1>;
  using Mat23io = Eigen::Matrix<Tio, 2, 3>;
  using Mat2Xio = Eigen::Matrix<Tio, 2, Eigen::Dynamic>;
  using Mat32io = Eigen::Matrix<Tio, 3, 2>;
  using Mat3Xio = Eigen::Matrix<Tio, 3, Eigen::Dynamic>;

  enum CameraModel { kUnknown = -1, kPinhole, kRadtan, kKB8 };
  enum UnProjectCurveType { kUnProject2Plane, kUnProject2Sphere };

  CameraBase(Tsize width, Tsize height, const vector<Tdata> &parameters)
      : width_(width), height_(height), parameters_(parameters) {}
  void SetUnProjectParams(UnProjectCurveType unptp, int max_iter, float prec) {
    unproject_type_ = unptp;
    num_max_iteration_ = max_iter;
    precision_ = prec;
  }
  virtual ~CameraBase() {}

  // *p_img here for g2o projectJac
  // inline func., hope to speed up
  inline virtual void Project(const Vec3io &p_3d, Vec2data *p_img, Mat23io *d_img_d_p3d = nullptr,
                              Mat2Xio *d_img_d_param = nullptr) const = 0;

  // input image 2d point, output 3d point on image coordinate(dir vector when unproject_type_ == kUnProject2Sphere)
  inline virtual void UnProject(const Vec2data &p_img, Vec3io *p_3d, Mat32io *d_p3d_d_img = nullptr,
                                Mat3Xio *d_p3d_d_param = nullptr) const = 0;

  // now only these funcions need width_/height_ filled
  // default(inline omitted but def in class{}) inline
  bool isInFrame(int x, int y, int boundary = 0, int level = 0) const {
    if (x >= boundary && x < width_ / (1 << level) - boundary && y >= boundary && y < height_ / (1 << level) - boundary)
      return true;
    return false;
  }
  Tsize width() const { return width_; }
  Tsize height() const { return height_; }

  const vector<Tdata> &GetParameters() const { return parameters_; }
  bool SetParamByIndex(const Tdata value, const size_t index) {
    assert(index < parameters_.size());
    parameters_[index] = value;
    return true;
  }

  const CameraModel &camera_model() const { return camera_model_; };

 protected:
  Tsize width_;
  Tsize height_;
  vector<Tdata> parameters_;

  UnProjectCurveType unproject_type_ = kUnProject2Plane;
  int num_max_iteration_ = 10;
  float precision_ = 1e-8;

  CameraModel camera_model_;
};

class GeometricCamera : public CameraBase, public GeometricCameraBase {
 protected:
  using Tcalc = FLT_CALC_CAMM;
  using Base = CameraBase;
  using BaseGeo = GeometricCameraBase;

  using BaseGeo::Tcb_;

 public:
  using Tdata = CameraBase::Tdata;
  using TdataGeo = GeometricCameraBase::Tdata;
  using Ptr = std::shared_ptr<GeometricCamera>;
  using size_t = std::size_t;

  template <typename _Tp>
  using vector = Base::vector<_Tp>;
  template <typename _Tp>
  using aligned_vector = Eigen::aligned_vector<_Tp>;
  template <typename _T1, typename _T2>
  using pair = std::pair<_T1, _T2>;
  using MapCamIdx2Idx = std::unordered_map<pair<size_t, size_t>, size_t, PairHash>;

  using typename Base::Tio;
  using typename Base::Tsize;
  using typename Base::Vec2data;
  using typename BaseGeo::CamId;
  using Vec2calc = Eigen::Matrix<Tcalc, 2, 1>;
  using Vec3calc = Eigen::Matrix<Tcalc, 3, 1>;
  using VecXdata = Eigen::Matrix<Tdata, Eigen::Dynamic, 1>;
  using Mat3data = Eigen::Matrix<Tdata, 3, 3>;
  using Mat3calc = Eigen::Matrix<Tcalc, 3, 3>;
  using Mat34calc = Eigen::Matrix<Tcalc, 3, 4>;
  using typename BaseGeo::SE3data;
  using SE3calc = Sophus::SE3<Tcalc>;
  using typename Base::Vec3io;
  using Mat3io = Eigen::Matrix3d;
  using Mat34io = Eigen::Matrix<Tio, 3, 4>;
  using SE3io = Sophus::SE3<Tio>;

  GeometricCamera(CamId id, Tsize width, Tsize height, const vector<Tdata> &parameters, const SE3data &Tbc)
      : Base(width, height, parameters), BaseGeo(id, Tbc) {}
  ~GeometricCamera() override{};

  inline virtual vector<Tdata> TriangulateMatches(const vector<const GeometricCamera *> &pcams_in,
                                                  const aligned_vector<Vec2data> &kps, const vector<float> &sigmaLevels,
                                                  Vec3io *p3D = nullptr, float thresh_cosdisparity = 0.9998,
                                                  const vector<vector<Tdata>> *purbf = nullptr,
                                                  const aligned_vector<SE3io> *pTwr = nullptr,
                                                  bool just_check_p3d = false) const;
  inline virtual bool epipolarConstrain(GeometricCamera *otherCamera, const Vec2data &kp1, const Vec2data &kp2,
                                        const Mat3io &R12, const Vec3io &t12, const float sigmaLevel, const float unc,
                                        bool bkp_distort = true) const;
  inline virtual bool FillMatchesFromPair(const vector<const GeometricCamera *> &pcams, size_t n_cams_tot,
                                          const vector<pair<size_t, size_t>> &vcamidx, float dist,
                                          vector<vector<size_t>> &vidxsmatches, vector<bool> &goodmatches_,
                                          MapCamIdx2Idx &mapcamidx2idxs, const float thresh_cosdisparity = 1. - 1.e-6,
                                          aligned_vector<Vec3io> *pv3dpoints = nullptr,
                                          aligned_vector<Vec2data> *pkpts = nullptr, vector<float> *psigmas = nullptr,
                                          vector<vector<float>> *plastdists = nullptr,
                                          int *pcount_descmatch = nullptr) const;

  const Tdata &fx() const { return Base::GetParameters()[0]; }
  const Tdata &fy() const { return Base::GetParameters()[1]; }
  const Tdata &cx() const { return Base::GetParameters()[2]; }
  const Tdata &cy() const { return Base::GetParameters()[3]; }
  inline virtual Mat3data toK() const = 0;

 protected:
  inline bool Triangulate(const aligned_vector<Vec2calc> &ps, const aligned_vector<Mat34io> &Tcws, Vec3io &x3D) const;
};

GeometricCamera::vector<GeometricCamera::Tdata> GeometricCamera::TriangulateMatches(
    const vector<const GeometricCamera *> &pcams_in, const aligned_vector<Vec2data> &kps,
    const vector<float> &sigmaLevels, Vec3io *p3D, float thresh_cosdisparity, const vector<vector<Tdata>> *purbf,
    const aligned_vector<SE3io> *pTwr, bool just_check_p3d) const {
  size_t n_cams = kps.size();
  vector<const GeometricCamera *> pcams;
  const vector<const GeometricCamera *> *ppcams = &pcams_in;
  if (pcams_in.size() == n_cams - 1) {
    pcams.push_back(this);
    pcams.insert(pcams.end(), pcams_in.begin(), pcams_in.end());
    ppcams = &pcams;
  } else if (pcams_in.size() != n_cams)
    return vector<Tdata>();
  aligned_vector<Vec3calc> normedcPs(n_cams);
  aligned_vector<SE3io> Twi(n_cams);  // if no pTwr, w means r here
  assert(!pTwr || n_cams == pTwr->size());
  for (size_t i = 0; i < n_cams; ++i) {
    auto &pcam = (*ppcams)[i];
    SE3io Twi_in = pcam->GetTrc().cast<Tio>();
    if (pTwr) Twi_in = (*pTwr)[i] * Twi_in;
    Twi[i] = Twi_in;
    Vec3io p3dtmp;
    pcam->UnProject(kps[i], &p3dtmp);
    normedcPs[i] = p3dtmp.cast<Tcalc>();
  }

  // Check parallax
  if (thresh_cosdisparity < 1.) {
    bool bret = true;
    for (size_t i = 0; i < n_cams - 1; ++i) {
      for (size_t j = i + 1; j < n_cams; ++j) {
        Eigen::Vector3d j2iP = Eigen::Vector3d(Twi[i].so3().inverse() * (Twi[j].so3() * normedcPs[j]));
        const float cosParallaxRays = normedcPs[i].dot(j2iP) / (normedcPs[i].norm() * j2iP.norm());
        if (cosParallaxRays <= thresh_cosdisparity) {
          bret = false;
          break;
        }
      }
      if (!bret) break;
    }
    if (bret) return vector<Tdata>();
  }

  // Parallax is good, so we try to triangulate
  Vec3io x3D;
  aligned_vector<Vec2calc> cPs(n_cams);
  aligned_vector<Mat34io> Tcws(n_cams);
  for (size_t i = 0; i < n_cams; ++i) {
    assert(1. == normedcPs[i](2));
    if (!just_check_p3d) cPs[i] = normedcPs[i].segment<2>(0);

    Tcws[i] = Twi[i].inverse().matrix3x4();
  }
  if (!just_check_p3d) {
    if (!Triangulate(cPs, Tcws, x3D)) return vector<Tdata>();
  } else {
    assert(p3D);
    x3D = *p3D;
  }
  vector<Tdata> czs(n_cams);
  assert(!purbf || n_cams == purbf->size());
  for (size_t i = 0; i < n_cams; ++i) {
    // Check positive depth
    const auto &Riw = Tcws[i].block<3, 3>(0, 0);
    const auto &tiw = Tcws[i].col(3);
    czs[i] = (Tdata)(Riw.row(2) * x3D + tiw(2));
    if (czs[i] <= 0) return vector<Tdata>();

    // Check reprojection error
    auto &pcam = (*ppcams)[i];
    Vec2data uv;
    pcam->Project((Riw * x3D + tiw).cast<Tio>(), &uv);
    VecXdata errs = uv - kps[i];
    float thresh_chi2 = 5.991;
    if (purbf && -1 != (*purbf)[i][0]) {
      errs.conservativeResize(3);
      Tdata u2_r = uv[0] - (*purbf)[i][1] / czs[i];
      errs[2] = u2_r - (*purbf)[i][0];
      thresh_chi2 = 7.8;
    }
    // Reprojection error is high
    if (errs.squaredNorm() > thresh_chi2 * sigmaLevels[i]) return vector<Tdata>();
  }

  if (p3D) *p3D = x3D;
  return czs;
}

bool GeometricCamera::epipolarConstrain(GeometricCamera *pCamera2, const Vec2data &kp1, const Vec2data &kp2,
                                        const Mat3io &R12in, const Vec3io &t12in, const float sigmaLevel,
                                        const float unc, bool bkp_distort) const {
  Vec3calc t12 = t12in.cast<Tcalc>();
  Mat3calc R12 = R12in.cast<Tcalc>();
  Mat3calc K1 = this->toK().cast<Tcalc>();
  Mat3calc K2 = pCamera2->toK().cast<Tcalc>();

//#define USE_DIR_EPI_ERR
#ifdef USE_DIR_EPI_ERR
  Vec3io pt1d, pt2d;
  if (bkp_distort) {
    this->UnProject(kp1, &pt1d);
    if (!pt1d.allFinite()) return false;
    pCamera2->UnProject(kp2, &pt2d);
    if (!pt2d.allFinite()) return false;
  } else {
    pt1d << kp1(0), kp1(1), 1;
    pt2d << kp2(0), kp2(1), 1;
    pt1d = K1.inverse() * pt1d;
    pt2d = K2.inverse() * pt2d;
  }
  Vec3calc pt1 = pt1d.normalized(), pt2 = pt2d.normalized();
  const Tdata invf = 4. / (K2(0, 0) + K2(1, 1) + K1(0, 0) + K1(1, 1)), invf2 = invf * invf;
  Tdata err2_epi_dir;
  // err2_epi_dir = pt1.transpose() * (t12.cross(R12 * pt2));
  // err2_epi_dir = err2_epi_dir * err2_epi_dir;
  // return err2_epi_dir < 5.991f * unc * invf2;

  Mat3calc R21 = R12.transpose();
  Vec3calc t21 = -R21 * t12;
  const Tcalc t21norm2 = t21.squaredNorm();
  if (t21norm2 < 1e-10) {
    err2_epi_dir = (R21 * pt1 - pt2).squaredNorm();
  } else {
    // it's easy to prove the cross pt 1' of the extension cord of vp and pt1 is
    // the min dist 2pts location then err_epi_dir means v/n2 - n1'
    const Vec3calc a = -t12;
    Vec3calc h = (a.cross(pt1)).normalized();
    Vec3calc aproj2n = a.dot(pt1) * pt1, B = a - aproj2n;
    Vec3calc v = R12 * pt2, vh = v.dot(h) * h, vp = v - vh;  // v is _1n2
    Vec3calc evp;
    const Tdata dmin = -0.02 / sqrt(1. * unc * invf2);  // 0;  //-INFINITY;  //
    // do not use auto on Eigen except u know what is done
    Vec3calc vpdmin = isfinite(dmin) ? (a + dmin * pt1).normalized() : dmin > 0 ? pt1 : -pt1;
    Vec3calc vp_vpdmin = vp - vpdmin;
    if (vp.dot(B) >= 0) {
      if (vpdmin.dot(pt1) <= vp.dot(pt1)) {  // || 1
        evp = vp - vp.normalized();
      } else {
        evp = vp_vpdmin;
      }
    } else {  // <0 min angle of v and n corresponds to dn =
              // -+infinity(now+>=-inf) pt., =0 can also here
      if (dmin < 0 && vpdmin.dot(pt1) <= -vp.dot(pt1)) {
        evp = -vp + vp.normalized();
        vh = -vh;
      } else {
        Vec3calc _vp_vpdmin = -vp - vpdmin, vp_vpdmax = vp - pt1, _vp_vpdmax = -vp - pt1;
        Tcalc norm_vp_vpd[4] = {vp_vpdmin.norm(), vp_vpdmax.norm(), _vp_vpdmin.norm(), _vp_vpdmax.norm()};
        int num_max_norm_vp_vpd = 4;
        if (isfinite(dmin) || dmin >= 0) num_max_norm_vp_vpd = 2;
        int min_pos = std::min_element(norm_vp_vpd, norm_vp_vpd + num_max_norm_vp_vpd) - norm_vp_vpd;
        if (min_pos < 2) {
          evp = !min_pos ? vp_vpdmin : vp_vpdmax;
        } else {
          evp = 2 == min_pos ? _vp_vpdmin : _vp_vpdmax;
          vh = -vh;
        }
      }
    }
    err2_epi_dir = (vh + evp).squaredNorm();
  }
  return err2_epi_dir < 5.991f * unc * invf2;
#else
  // Compute Fundamental Matrix F12=K1^(-T)*t12^R12*K2^(-1)
  Mat3calc F12 = K1.transpose().inverse() * Sophus::SO3ex<Tcalc>::hat(t12) * R12 * K2.inverse();

  // Epipolar line in second image l = x1'F12 = [a b c], or l2=e2 cross
  // x2=F21*x1=[a;b;c](easy to prove F21'=F12), here l2 means n
  // vector(perpendicular to x2&&e2), e2 means epipolar point in 2nd image
  Vec2calc pt1, pt2;
  if (bkp_distort) {
    Vec3io pt1d, pt2d;
    this->UnProject(kp1, &pt1d);
    pt1d = K1 * pt1d;
    if (!pt1d.allFinite()) return false;
    pCamera2->UnProject(kp2, &pt2d);
    pt2d = K2 * pt2d;
    if (!pt2d.allFinite()) return false;
    Tcalc invz = 1. / pt1d[2];
    pt1 << pt1d[0] * invz, pt1d[1] * invz;
    invz = 1. / pt2d[2];
    pt2 << pt2d[0] * invz, pt2d[1] * invz;
  } else {
    pt1 = kp1.cast<Tcalc>();
    pt2 = kp2.cast<Tcalc>();
  }
  const Tdata a = pt1[0] * F12(0, 0) + pt1[1] * F12(1, 0) + F12(2, 0);  // p1'*F12
  const Tdata b = pt1[0] * F12(0, 1) + pt1[1] * F12(1, 1) + F12(2, 1);
  const Tdata c = pt1[0] * F12(0, 2) + pt1[1] * F12(1, 2) + F12(2, 2);

  // here norm(n)==|n|
  // p1'*F12*p2==num near 0, or this dot result is |dist|*cos(theta)*|n|(imagine
  // a plane(x2&&e2) with a point(p2) |dist|cos(theta) away) theta is the angle
  // between dist vector and n vector
  const Tdata num = a * pt2[0] + b * pt2[1] + c;

  const Tdata den = a * a + b * b;  // this nx^2+ny^2 is the projection of n, or
                                    // it's (norm(n)*cos(theta))^2

  if (den == 0) return false;

  // here is the |dist|^2, dist vector is the distance vector pointing to x2
  // from the epipolar line
  const float dsqr = num * num / den;

  return dsqr < 3.84f * unc;  // 2sigma rule;1.96^2,95.45%
#endif
}

bool GeometricCamera::FillMatchesFromPair(const vector<const GeometricCamera *> &pcams_in, size_t n_cams_tot,
                                          const vector<pair<size_t, size_t>> &vcamidx, float dist,
                                          vector<vector<size_t>> &vidxsmatches, vector<bool> &goodmatches,
                                          MapCamIdx2Idx &mapcamidx2idxs, const float thresh_cosdisparity,
                                          aligned_vector<Vec3io> *pv3dpoints, aligned_vector<Vec2data> *pkpts,
                                          vector<float> *psigmas, vector<vector<float>> *plastdists,
                                          int *pcount_descmatch) const {
  using std::get;
  using std::make_pair;
  size_t n_cams = vcamidx.size();
  vector<const GeometricCamera *> pcams;
  const vector<const GeometricCamera *> *ppcams = &pcams_in;
  if (pcams_in.size() == n_cams - 1) {
    pcams.push_back(this);
    pcams.insert(pcams.end(), pcams_in.begin(), pcams_in.end());
    ppcams = &pcams;
  } else if (pcams_in.size() != n_cams)
    return false;
  auto &idxi = get<1>(vcamidx[0]), &idxj = get<1>(vcamidx[1]);
  auto &cami = get<0>(vcamidx[0]), &camj = get<0>(vcamidx[1]);
  auto camidxi = make_pair(cami, idxi), camidxj = make_pair(camj, idxj);
  assert(cami < n_cams_tot);
  assert(camj < n_cams_tot);
  auto iteri = mapcamidx2idxs.find(camidxi), iterj = mapcamidx2idxs.find(camidxj);
  if (iteri == mapcamidx2idxs.end() && iterj != mapcamidx2idxs.end()) {
    iteri = iterj;
  }
  uint8_t checkdepth[2] = {0};  // 1 means create, 2 means replace
  size_t ididxs;
  uint8_t contradict = 0;
  if (iteri != mapcamidx2idxs.end()) {
    ididxs = iteri->second;
    contradict = (iterj != mapcamidx2idxs.end() && iterj->second != ididxs) ? 2 : 0;
    auto idxs = vidxsmatches[ididxs];
#ifdef USE_STRATEGY_MIN_DIST
    if (contradict && plastdists) {
      auto &lastdists = *plastdists;
      auto &idxsj = vidxsmatches[iterj->second];
      float dists_sum[2] = {0, 0};
      size_t count_num[2] = {0, 0};
      for (size_t itmp = 0; itmp < n_cams_tot; ++itmp) {
        if (-1 != idxs[itmp]) {
          dists_sum[0] += lastdists[ididxs][itmp];
          ++count_num[0];
        }
        if (-1 != idxsj[itmp]) {
          dists_sum[1] += lastdists[iterj->second][itmp];
          ++count_num[1];
        }
      }
      if (dists_sum[1] * count_num[0] < dists_sum[0] * count_num[1]) {
        idxs = idxsj;
        ididxs = iterj->second;
        contradict = 1;
      }
    }
#endif
    if (-1 == idxs[cami]
#ifdef USE_STRATEGY_MIN_DIST
        // idxi!= for 1<->2,1<->3, 2<->3 will be skipped
        || (plastdists && idxi != idxs[cami] && (*plastdists)[ididxs][cami] > dist)
#endif
    ) {
      checkdepth[0] = 2;
    }
#ifdef USE_STRATEGY_ABANDON
    else if (idxi != idxs[cami])
      goodmatches[ididxs] = false;
#endif
    if (-1 == idxs[camj]
#ifdef USE_STRATEGY_MIN_DIST
        || (plastdists && idxj != idxs[camj] && (*plastdists)[ididxs][camj] > dist)
#endif
    ) {
      checkdepth[1] = 2;
    }
#ifdef USE_STRATEGY_ABANDON
    else if (idxj != idxs[camj])
      goodmatches[ididxs] = false;
    if (contradict) goodmatches[ididxs] = false;
#endif
  } else {
    checkdepth[0] = 1;
    checkdepth[1] = 1;
  }
  if (pcount_descmatch) ++*pcount_descmatch;
  if (checkdepth[0] || checkdepth[1]) {
#ifndef USE_STRATEGY_MIN_DIST
    if (contradict) return false;
#endif
    Vec3io p3D;
    bool bdepth_ok = !psigmas || !pkpts;
    if (!bdepth_ok) {
      auto depths = (*ppcams)[0]->TriangulateMatches(vector<const GeometricCamera *>(1, (*ppcams)[1]), *pkpts, *psigmas,
                                                     &p3D, thresh_cosdisparity);
      if (depths.empty()) {
        // cout << "dpeth emtpy" << endl;
        return false;
      }
      // cout << "dp21=" << depths[1] << " " << depths[0] << endl;
      if (depths[0] > 0.0001f && depths[1] > 0.0001f) bdepth_ok = true;
    }
    if (bdepth_ok) {
      if (1 == checkdepth[0]) {
        assert(1 == checkdepth[1]);
        vector<size_t> idxs(n_cams_tot, -1);
        idxs[cami] = idxi;
        idxs[camj] = idxj;
        ididxs = vidxsmatches.size();
        mapcamidx2idxs.emplace(camidxi, ididxs);
        mapcamidx2idxs.emplace(camidxj, ididxs);
        vidxsmatches.push_back(idxs);
        if (pv3dpoints) pv3dpoints->resize(vidxsmatches.size());
        goodmatches.push_back(true);
#ifdef USE_STRATEGY_MIN_DIST
        if (plastdists) {
          vector<float> dists(n_cams_tot, INFINITY);
          dists[cami] = dist;
          dists[camj] = dist;
          plastdists->push_back(dists);
        }
#endif
      }
#ifdef USE_STRATEGY_MIN_DIST
      else if (2 == checkdepth[0] || 2 == checkdepth[1]) {
        if (contradict) {
          auto ididxs_contradict = 1 == contradict ? iteri->second : iterj->second;
          auto &idxs = vidxsmatches[ididxs_contradict];
          if (idxi == idxs[cami]) {
            mapcamidx2idxs.erase(camidxi);
            if (plastdists) (*plastdists)[ididxs_contradict][cami] = INFINITY;
            idxs[cami] = -1;
          }
          if (idxj == idxs[camj]) {
            mapcamidx2idxs.erase(camidxj);
            if (plastdists) (*plastdists)[ididxs_contradict][camj] = INFINITY;
            idxs[camj] = -1;
          }
        }
        auto &idxs = vidxsmatches[ididxs];
        if (2 == checkdepth[0]) {
          if (idxi != idxs[cami]) {
            if (-1 != idxs[cami]) mapcamidx2idxs.erase(make_pair(cami, idxs[cami]));
            mapcamidx2idxs.emplace(camidxi, ididxs);
            idxs[cami] = idxi;
          }
          if (plastdists) (*plastdists)[ididxs][cami] = dist;
        } else if (plastdists && (*plastdists)[ididxs][cami] > dist)
          (*plastdists)[ididxs][cami] = dist;
        if (2 == checkdepth[1]) {
          if (idxj != idxs[camj]) {
            if (-1 != idxs[camj]) mapcamidx2idxs.erase(make_pair(camj, idxs[camj]));
            mapcamidx2idxs.emplace(camidxj, ididxs);
            idxs[camj] = idxj;
          }
          if (plastdists) (*plastdists)[ididxs][camj] = dist;
        } else if (plastdists && (*plastdists)[ididxs][camj] > dist)
          (*plastdists)[ididxs][camj] = dist;
      }
#endif
      if (pv3dpoints) (*pv3dpoints)[ididxs] = p3D;  // here should return pt in cami's ref frame, usually 0
    } else
      return false;
  } else
    return false;
  return true;
}

bool GeometricCamera::Triangulate(const aligned_vector<Vec2calc> &ps, const aligned_vector<Mat34io> &Tcws,
                                  Vec3io &x3D) const {
  typedef Eigen::Matrix<Tcalc, 4, 1> Vector4calc;
  typedef Eigen::Matrix<Tcalc, Eigen::Dynamic, 4> MatrixX4calc;
  typedef Eigen::Matrix<Tcalc, Eigen::Dynamic, Eigen::Dynamic> MatrixXXcalc;
  size_t npts = ps.size();
  const size_t dim_b = npts * 2;
  constexpr size_t dim_x = 4;
  assert(npts == Tcws.size());
  MatrixX4calc A(dim_b, dim_x);

  // Linear Triangulation Method, though it's not the best method
  // Xc=K^(-1)*P=1./d*[Rcw|tcw]*Xw;(Xc*d-[Rcw|tcw]*Xw)(0:1),d=([Rcw|tcw]*Xw)(2)=Tcw.row(2)*Xw
  //=>A=[Xc1(0)*Tc1w.row(2)-Tc1w.row(0);Xc1(1)*Tc1w.row(2)-Tc1w.row(1);Xc2(0)*Tc2w.row(2)-Tc2w.row(0);Xc2(1)*Tc2w.row(2)-Tc2w.row(1)]=4*4
  // matrix, AX=0, see http://www.robots.ox.ac.uk/~az/tutorials/tutoriala.pdf
  A.setZero();
  for (size_t i = 0; i < npts; ++i) {
    A.row(i * 2) = ps[i].x() * Tcws[i].row(2) - Tcws[i].row(0);
    A.row(i * 2 + 1) = ps[i].y() * Tcws[i].row(2) - Tcws[i].row(1);
  }

  // min(X) ||AX||^2 s.t. ||x||=1 should use SVD method, see
  // http://blog.csdn.net/zhyh1435589631/article/details/62218421
  Eigen::JacobiSVD<MatrixXXcalc> svdA(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Vector4calc &x4D = svdA.matrixV().col(3);
  if (!x4D(3)) return false;
  x3D = x4D.segment<3>(0) / x4D(3);
  //  cv::Mat u, w, vt;
  //  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  //  x3D = vt.row(3).t();
  //  x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
  return true;
}

using GeneralCamera = GeometricCameraBase;
using Camera = GeometricCamera;

class MultiCamerasBase {  // designed for frame
 public:
  using Ptr = std::shared_ptr<MultiCamerasBase>;

  template <typename _Tp>
  using vector = std::vector<_Tp>;

  MultiCamerasBase() {}
  MultiCamerasBase(const vector<GeneralCamera::Ptr> &cams_init) : general_cams_(cams_init) {}
  MultiCamerasBase(const vector<Camera::Ptr> &cams_init) {
    size_t n_cams = cams_init.size();
    general_cams_.resize(n_cams);
    for (int i = 0; i < n_cams; ++i) general_cams_[i] = cams_init[i];
  }
  virtual ~MultiCamerasBase() {}

  const vector<GeneralCamera::Ptr> &GetGeneralCamerasPtr() const { return general_cams_; }
  const GeneralCamera::Ptr &GetGeneralCameraPtr(size_t i = 0) const { return general_cams_[i]; }

 protected:
  // GeneralCameraUsedPtr here KeyFrame must be deep copied from Frame, for Tbc
  // may be changed in KF, where mutex needed
  vector<GeneralCamera::Ptr> general_cams_;
};

class MultiCameras : public MultiCamerasBase {  // designed for cams frame
  using Base = MultiCamerasBase;

 public:
  using Ptr = std::shared_ptr<MultiCameras>;

  template <typename _Tp>
  using vector = Base::vector<_Tp>;

  MultiCameras() {}
  MultiCameras(const vector<Camera::Ptr> &cams_init) : Base(cams_init), cams_(cams_init) {}
  ~MultiCameras() override {}

  const vector<Camera::Ptr> &GetCamerasPtr() const { return cams_; }
  const Camera::Ptr &GetCameraPtr(size_t i = 0) const { return cams_[i]; }

 protected:
  // CameraPtr here KeyFrame must be deep copied from Frame, for Tbc may be
  // changed in KF, where mutex needed
  // we don't directly use general_cams_ for efficiency when needing cams_
  vector<Camera::Ptr> cams_;
};

}  // namespace camm
}  // namespace VIEO_SLAM
