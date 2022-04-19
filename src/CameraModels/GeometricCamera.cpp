//
// Created by leavesnight on 2021/12/7.
//

#include "GeometricCamera.h"
#include "Converter.h"
#include "so3_extra.h"
#include <string>

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

void GeometricCamera::Triangulate(const aligned_vector<Eigen::Vector2d> &ps, const aligned_vector<Matrix34> &Tcws,
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
  x3D = x4D.segment<3>(0) / x4D(3);
  //  cv::Mat u, w, vt;
  //  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  //  x3D = vt.row(3).t();
  //  x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

GeometricCamera::vector<float> GeometricCamera::TriangulateMatches(vector<GeometricCamera *> pCamerasOther,
                                                                   const vector<cv::KeyPoint> &kps,
                                                                   const vector<float> &sigmaLevels, cv::Mat *p3D,
                                                                   double thresh_cosdisparity) {
  size_t n_cams = pCamerasOther.size() + 1;
  CV_Assert(n_cams == kps.size());
  aligned_vector<Eigen::Vector3d> normedcPs(n_cams);
  vector<Eigen::Matrix3d> R1i(n_cams);
  vector<Eigen::Vector3d> t1i(n_cams);
  GeometricCamera *pcam = this;
  for (size_t i = 0, iother; i < n_cams; iother = i++) {
    if (i) {
      pcam = pCamerasOther[iother];
      R1i[iother] = Rcr_ * pcam->Rcr_.transpose();
      t1i[iother] = Rcr_ * (-pcam->Rcr_.transpose() * pcam->tcr_) + tcr_;
    }
    normedcPs[i] = pcam->unproject(Eigen::Vector2d(kps[i].pt.x, kps[i].pt.y));
  }

  // Check parallax
  bool bret = true;
  for (size_t i = 0, iother = -1; i < n_cams - 1; iother = i++) {
    for (size_t j = i + 1, jother = i; j < n_cams; jother = j++) {
      Eigen::Vector3d j2iP = (-1 == iother) ? R1i[jother] * normedcPs[j]
                                            : Eigen::Vector3d(R1i[iother].transpose() * R1i[jother] * normedcPs[j]);
      const float cosParallaxRays = normedcPs[i].dot(j2iP) / (normedcPs[i].norm() * j2iP.norm());
      if (cosParallaxRays <= thresh_cosdisparity) {
        bret = false;
        break;
      }
    }
    if (!bret) break;
  }
  if (bret) return vector<float>();

  // Parallax is good, so we try to triangulate
  Eigen::Vector3d x3D;
  aligned_vector<Eigen::Vector2d> cPs(n_cams);
  aligned_vector<Matrix34> Tcws(n_cams);
  for (size_t i = 0, iother; i < n_cams; iother = i++) {
    CV_Assert(1. == normedcPs[i](2));
    cPs[i] = normedcPs[i].segment<2>(0);
    if (!i)
      Tcws[i] = Matrix34::Identity();
    else {
      Tcws[i].block<3, 3>(0, 0) = R1i[iother].transpose();
      Tcws[i].col(3) = -Tcws[i].block<3, 3>(0, 0) * t1i[iother];
    }
  }
  Triangulate(cPs, Tcws, x3D);
  vector<float> czs(n_cams);
  for (size_t i = 0, iother; i < n_cams; iother = i++) {
    // Check positive depth
    const auto &Ri1 = Tcws[i].block<3, 3>(0, 0);
    const auto &ti1 = Tcws[i].col(3);
    czs[i] = Ri1.row(2) * x3D + ti1(2);
    if (czs[i] <= 0) return vector<float>();

    // Check reprojection error
    if (i)
      pcam = pCamerasOther[iother];
    else
      pcam = this;
    Eigen::Vector2d uv = pcam->project(Ri1 * x3D + ti1);
    double errs[2] = {uv[0] - kps[i].pt.x, uv[1] - kps[i].pt.y};
    // Reprojection error is high
    if (errs[0] * errs[0] + errs[1] * errs[1] > 5.991 * sigmaLevels[i]) return vector<float>();
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