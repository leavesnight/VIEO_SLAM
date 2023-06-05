//
// Created by leavesnight on 2021/12/7.
//

#include "GeometricCamera.h"
#include "Converter.h"
#include "common/so3_extra.h"
#include <string>
#include "common/config.h"

using std::cout;
using std::endl;
using std::string;
using std::to_string;

namespace VIEO_SLAM {
cv::Point2f GeometricCamera::project(const cv::Point3f &p3D) {
  auto pteig = project(Eigen::Vector3d(p3D.x, p3D.y, p3D.z));
  return cv::Point2f(pteig[0], pteig[1]);
}
cv::Point2f GeometricCamera::project(const cv::Mat &m3D) {
  const float *p3D = m3D.ptr<float>();
  auto pteig = project(Eigen::Vector3d(p3D[0], p3D[1], p3D[2]));
  return cv::Point2f(pteig[0], pteig[1]);
}

cv::Point3f GeometricCamera::unproject(const cv::Point2f &p2D) {
  auto normedPeig = unproject(Eigen::Vector2d(p2D.x, p2D.y));
  return cv::Point3f(normedPeig[0], normedPeig[1], normedPeig[2]);
}
cv::Mat GeometricCamera::unprojectMat(const cv::Point2f &p2D) {
  auto normedPeig = unproject(Eigen::Vector2d(p2D.x, p2D.y));
  return (cv::Mat_<float>(3, 1) << normedPeig[0], normedPeig[1], normedPeig[2]);
}

// cv::Mat GeometricCamera::projectJac(const cv::Point3f &p3D) {
//  cv::Mat Jac(2, 3, CV_32F);
//  Eigen::Matrix<double, 2, 3> jac = projectJac(Eigen::Vector3d(p3D.x, p3D.y, p3D.z));
//  for (int i = 0; i < 2; ++i)
//    for (int j = 0; j < 3; ++j) Jac.at<float>(i, j) = jac(i, j);
//  return Jac;
//}

bool GeometricCamera::Triangulate(const aligned_vector<Eigen::Vector2d> &ps, const aligned_vector<Matrix34> &Tcws,
                                  Eigen::Vector3d &x3D) {
  typedef double Tcalc;
  typedef Eigen::Matrix<Tcalc, Eigen::Dynamic, 4> MatrixX4calc;
  typedef Eigen::Matrix<Tcalc, Eigen::Dynamic, Eigen::Dynamic> MatrixXXcalc;
  size_t npts = ps.size();
  const size_t dim_b = npts * 2;
  constexpr size_t dim_x = 4;
  CV_Assert(npts == Tcws.size());
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
  const auto &x4D = svdA.matrixV().col(3);
  if (!x4D(3)) return false;
  x3D = x4D.segment<3>(0) / x4D(3);
  //  cv::Mat u, w, vt;
  //  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  //  x3D = vt.row(3).t();
  //  x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
  return true;
}

GeometricCamera::vector<float> GeometricCamera::TriangulateMatches(
    const vector<GeometricCamera *> &pcams_in, const aligned_vector<Eigen::Vector2d> &kps,
    const vector<float> &sigmaLevels, cv::Mat *p3D, double thresh_cosdisparity, const vector<vector<float>> *purbf,
    const aligned_vector<Sophus::SE3d> *pTwr, bool just_check_p3d) {
  size_t n_cams = kps.size();
  vector<GeometricCamera *> pcams;
  const vector<GeometricCamera *> *ppcams = &pcams_in;
  if (pcams_in.size() == n_cams - 1) {
    pcams.push_back(this);
    pcams.insert(pcams.end(), pcams_in.begin(), pcams_in.end());
    ppcams = &pcams;
  } else if (pcams_in.size() != n_cams)
    return vector<float>();
  aligned_vector<Eigen::Vector3d> normedcPs(n_cams);
  vector<Eigen::Matrix3d> Rwi(n_cams);  // if no pTwr, w means r here
  vector<Eigen::Vector3d> twi(n_cams);
  CV_Assert(!pTwr || n_cams == pTwr->size());
  for (size_t i = 0; i < n_cams; ++i) {
    auto &pcam = (*ppcams)[i];
    auto Twi = pcam->GetTcr().inverse();  // Tri
    if (pTwr) Twi = (*pTwr)[i] * Twi;
    Rwi[i] = Twi.rotationMatrix();
    twi[i] = Twi.translation();
    normedcPs[i] = pcam->unproject(kps[i]);
  }

  // Check parallax
  if (thresh_cosdisparity < 1.) {
    bool bret = true;
    for (size_t i = 0; i < n_cams - 1; ++i) {
      for (size_t j = i + 1; j < n_cams; ++j) {
        Eigen::Vector3d j2iP = Eigen::Vector3d(Rwi[i].transpose() * Rwi[j] * normedcPs[j]);
        const float cosParallaxRays = normedcPs[i].dot(j2iP) / (normedcPs[i].norm() * j2iP.norm());
        if (cosParallaxRays <= thresh_cosdisparity) {
          bret = false;
          break;
        }
      }
      if (!bret) break;
    }
    if (bret) return vector<float>();
  }

  // Parallax is good, so we try to triangulate
  Eigen::Vector3d x3D;
  aligned_vector<Eigen::Vector2d> cPs(n_cams);
  aligned_vector<Matrix34> Tcws(n_cams);
  for (size_t i = 0; i < n_cams; ++i) {
    CV_Assert(1. == normedcPs[i](2));
    if (!just_check_p3d) cPs[i] = normedcPs[i].segment<2>(0);

    Tcws[i].block<3, 3>(0, 0) = Rwi[i].transpose();
    Tcws[i].col(3) = -Tcws[i].block<3, 3>(0, 0) * twi[i];
  }
  if (!just_check_p3d) {
    if (!Triangulate(cPs, Tcws, x3D)) return vector<float>();
  }
  vector<float> czs(n_cams);
  CV_Assert(!purbf || n_cams == purbf->size());
  for (size_t i = 0; i < n_cams; ++i) {
    // Check positive depth
    const auto &Riw = Tcws[i].block<3, 3>(0, 0);
    const auto &tiw = Tcws[i].col(3);
    czs[i] = Riw.row(2) * x3D + tiw(2);
    if (czs[i] <= 0) return vector<float>();

    // Check reprojection error
    auto &pcam = (*ppcams)[i];
    Eigen::Vector2d uv = pcam->project(Riw * x3D + tiw);
    Eigen::VectorXd errs = uv - kps[i];
    double thresh_chi2 = 5.991;
    if (purbf && -1 != (*purbf)[i][0]) {
      errs.conservativeResize(3);
      float u2_r = uv[0] - (*purbf)[i][1] / czs[i];
      errs[2] = u2_r - (*purbf)[i][0];
      thresh_chi2 = 7.8;
    }
    // Reprojection error is high
    if (errs.squaredNorm() > thresh_chi2 * sigmaLevels[i]) return vector<float>();
  }

  if (p3D) *p3D = Converter::toCvMat(x3D);
  return czs;
}

// float GeometricCamera::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) { return 1.0; }

cv::Mat GeometricCamera::toKcv() { return Converter::toCvMat(toK()); }
bool GeometricCamera::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                        const cv::Mat &R12in, const cv::Mat &t12in, const float sigmaLevel,
                                        const float unc, bool bkp_distort) {
  // Compute Fundamental Matrix F12=K1^(-T)*t12^R12*K2^(-1)
  Eigen::Vector3d t12 = Converter::toVector3d(t12in);
  Eigen::Matrix3d R12 = Converter::toMatrix3d(R12in);
  auto K1 = this->toK();
  auto K2 = pCamera2->toK();
  Eigen::Matrix3d F12 = K1.transpose().inverse() * Sophus::SO3exd::hat(t12) * R12 * K2.inverse();

  // Epipolar line in second image l = x1'F12 = [a b c], or l2=e2 cross x2=F21*x1=[a;b;c](easy to prove F21'=F12), here
  // l2 means n vector(perpendicular to x2&&e2), e2 means epipolar point in 2nd image
  cv::Point2f pt1 = kp1.pt, pt2 = kp2.pt;
  if (bkp_distort) {
    auto pt1normed = unproject(kp1.pt), pt2normed = pCamera2->unproject(kp2.pt);
    Eigen::Vector3d pt = K1 * Eigen::Vector3d(pt1normed.x, pt1normed.y, pt1normed.z);
    if (!pt(2)) return false;
    pt /= pt(2);
    pt1.x = pt.x();
    pt1.y = pt.y();
    pt = K2 * Eigen::Vector3d(pt2normed.x, pt2normed.y, pt2normed.z);
    if (!pt(2)) return false;
    pt /= pt(2);
    pt2.x = pt.x();
    pt2.y = pt.y();
  }
  const float a = pt1.x * F12(0, 0) + pt1.y * F12(1, 0) + F12(2, 0);  // p1'*F12
  const float b = pt1.x * F12(0, 1) + pt1.y * F12(1, 1) + F12(2, 1);
  const float c = pt1.x * F12(0, 2) + pt1.y * F12(1, 2) + F12(2, 2);

  // here norm(n)==|n|
  // p1'*F12*p2==num near 0, or this dot result is |dist|*cos(theta)*|n|(imagine a
  // plane(x2&&e2) with a point(p2) |dist|cos(theta) away)
  // theta is the angle between dist vector and n vector
  const float num = a * pt2.x + b * pt2.y + c;

  const float den = a * a + b * b;  // this nx^2+ny^2 is the projection of n, or it's (norm(n)*cos(theta))^2

  if (den == 0) return false;

  // here is the |dist|^2, dist vector is the distance vector pointing to x2 from the epipolar line
  const float dsqr = num * num / den;

  return dsqr < 3.84 * unc;  // 2sigma rule;1.96^2,95.45%
}

bool GeometricCamera::FillMatchesFromPair(const vector<GeometricCamera *> &pcams_in, size_t n_cams_tot,
                                          const vector<std::pair<size_t, size_t>> &vcamidx, double dist,
                                          vector<vector<size_t>> &mvidxsMatches, vector<bool> &goodmatches,
                                          std::map<std::pair<size_t, size_t>, size_t> &mapcamidx2idxs_,
                                          const double thresh_cosdisparity, aligned_vector<Eigen::Vector3d> *pv3dpoints,
                                          const aligned_vector<Eigen::Vector2d> *pkpts, const vector<float> *psigmas,
                                          vector<vector<double>> *plastdists, int *pcount_descmatch) {
  using std::get;
  using std::make_pair;
  size_t n_cams = vcamidx.size();
  vector<GeometricCamera *> pcams;
  const vector<GeometricCamera *> *ppcams = &pcams_in;
  if (pcams_in.size() == n_cams - 1) {
    pcams.push_back(this);
    pcams.insert(pcams.end(), pcams_in.begin(), pcams_in.end());
    ppcams = &pcams;
  } else if (pcams_in.size() != n_cams)
    return false;
  auto &idxi = get<1>(vcamidx[0]), &idxj = get<1>(vcamidx[1]);
  auto &cami = get<0>(vcamidx[0]), &camj = get<0>(vcamidx[1]);
  auto camidxi = make_pair(cami, idxi), camidxj = make_pair(camj, idxj);
  CV_Assert(cami < n_cams_tot);
  CV_Assert(camj < n_cams_tot);
  auto iteri = mapcamidx2idxs_.find(camidxi), iterj = mapcamidx2idxs_.find(camidxj);
  if (iteri == mapcamidx2idxs_.end() && iterj != mapcamidx2idxs_.end()) {
    iteri = iterj;
  }
  uint8_t checkdepth[2] = {0};  // 1 means create, 2 means replace
  size_t ididxs;
  uint8_t contradict = 0;
  if (iteri != mapcamidx2idxs_.end()) {
    ididxs = iteri->second;
    contradict = (iterj != mapcamidx2idxs_.end() && iterj->second != ididxs) ? 2 : 0;
    auto idxs = mvidxsMatches[ididxs];
#ifdef USE_STRATEGY_MIN_DIST
    if (contradict && plastdists) {
      auto &lastdists = *plastdists;
      auto &idxsj = mvidxsMatches[iterj->second];
      double dists_sum[2] = {0, 0};
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
    cv::Mat p3D;
    bool bdepth_ok = !psigmas || !pkpts;
    if (!bdepth_ok) {
      auto depths = (*ppcams)[0]->TriangulateMatches(vector<GeometricCamera *>(1, (*ppcams)[1]), *pkpts, *psigmas, &p3D,
                                                     thresh_cosdisparity);
      if (depths.empty()) {
        // cout << "dpeth emtpy" << endl;
        return false;
      }
      // cout << "dp21=" << depths[1] << " " << depths[0] << endl;
      if (depths[0] > 0.0001f && depths[1] > 0.0001f) bdepth_ok = true;
    }
    if (bdepth_ok) {
      if (1 == checkdepth[0]) {
        CV_Assert(1 == checkdepth[1]);
        vector<size_t> idxs(n_cams_tot, -1);
        idxs[cami] = idxi;
        idxs[camj] = idxj;
        ididxs = mvidxsMatches.size();
        mapcamidx2idxs_.emplace(camidxi, ididxs);
        mapcamidx2idxs_.emplace(camidxj, ididxs);
        mvidxsMatches.push_back(idxs);
        if (pv3dpoints) pv3dpoints->resize(mvidxsMatches.size());
        goodmatches.push_back(true);
#ifdef USE_STRATEGY_MIN_DIST
        if (plastdists) {
          vector<double> dists(n_cams_tot, INFINITY);
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
          auto &idxs = mvidxsMatches[ididxs_contradict];
          if (idxi == idxs[cami]) {
            mapcamidx2idxs_.erase(camidxi);
            if (plastdists) (*plastdists)[ididxs_contradict][cami] = INFINITY;
            idxs[cami] = -1;
          }
          if (idxj == idxs[camj]) {
            mapcamidx2idxs_.erase(camidxj);
            if (plastdists) (*plastdists)[ididxs_contradict][camj] = INFINITY;
            idxs[camj] = -1;
          }
        }
        auto &idxs = mvidxsMatches[ididxs];
        if (2 == checkdepth[0]) {
          if (idxi != idxs[cami]) {
            if (-1 != idxs[cami]) mapcamidx2idxs_.erase(make_pair(cami, idxs[cami]));
            mapcamidx2idxs_.emplace(camidxi, ididxs);
            idxs[cami] = idxi;
          }
          if (plastdists) (*plastdists)[ididxs][cami] = dist;
        } else if (plastdists && (*plastdists)[ididxs][cami] > dist)
          (*plastdists)[ididxs][cami] = dist;
        if (2 == checkdepth[1]) {
          if (idxj != idxs[camj]) {
            if (-1 != idxs[camj]) mapcamidx2idxs_.erase(make_pair(camj, idxs[camj]));
            mapcamidx2idxs_.emplace(camidxj, ididxs);
            idxs[camj] = idxj;
          }
          if (plastdists) (*plastdists)[ididxs][camj] = dist;
        } else if (plastdists && (*plastdists)[ididxs][camj] > dist)
          (*plastdists)[ididxs][camj] = dist;
      }
#endif
      if (pv3dpoints)
        (*pv3dpoints)[ididxs] =
            Converter::toVector3d(p3D.clone());  // here should return pt in cami's ref frame, usually 0
    }
  } else
    return false;
  return true;
}

const cv::Mat GeometricCamera::Getcvtrc() { return Converter::toCvMat(GetTrc().translation()); }

//    bool GeometricCamera::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const
//    std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
//                                 cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool>
//                                 &vbTriangulated){
//        if(!tvr){
//            cv::Mat K = this->toKcv();
//            tvr = new TwoViewReconstruction(K);
//        }
//
//        return tvr->Reconstruct(vKeys1,vKeys2,vMatches12,R21,t21,vP3D,vbTriangulated);
//    }
}  // namespace VIEO_SLAM
