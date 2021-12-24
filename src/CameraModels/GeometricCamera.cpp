//
// Created by leavesnight on 2021/12/7.
//

#include "GeometricCamera.h"
#include "Converter.h"
#include "so3_extra.h"
#include <string>
#include <iostream>
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

//cv::Mat GeometricCamera::projectJac(const cv::Point3f &p3D) {
//  cv::Mat Jac(2, 3, CV_32F);
//  Eigen::Matrix<double, 2, 3> jac = projectJac(Eigen::Vector3d(p3D.x, p3D.y, p3D.z));
//  for (int i = 0; i < 2; ++i)
//    for (int j = 0; j < 3; ++j) Jac.at<float>(i, j) = jac(i, j);
//  return Jac;
//}

void GeometricCamera::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Mat &Tcw1,
                                  const cv::Mat &Tcw2, cv::Mat &x3D) {
  cv::Mat A(4, 4, CV_32F);

  A.row(0) = p1.x * Tcw1.row(2) - Tcw1.row(0);
  A.row(1) = p1.y * Tcw1.row(2) - Tcw1.row(1);
  A.row(2) = p2.x * Tcw2.row(2) - Tcw2.row(0);
  A.row(3) = p2.y * Tcw2.row(2) - Tcw2.row(1);

  cv::Mat u, w, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  x3D = vt.row(3).t();
  x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

// TODO: change to 4 cams
float GeometricCamera::TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                          const float sigmaLevel, const float unc, cv::Mat &p3D, float *pz2) {
  cv::Mat r1 = this->unprojectMat(kp1.pt);
  cv::Mat r2 = pCamera2->unprojectMat(kp2.pt);

  cv::Mat Rr1T = Trc_.rowRange(0, 3).colRange(0, 3).t();
  cv::Mat R12 = Rr1T * pCamera2->Trc_.rowRange(0, 3).colRange(0, 3);
  cv::Mat t12 = Rr1T * pCamera2->Trc_.col(3) - Rr1T * Trc_.col(3);
  // Check parallax
  cv::Mat r21 = R12 * r2;

  const float cosParallaxRays = r1.dot(r21) / (cv::norm(r1) * cv::norm(r21));

  if (cosParallaxRays > 0.9998) {
    return -1;
  }

  // Parallax is good, so we try to triangulate
  cv::Point2f p11, p22;
  const float *pr1 = r1.ptr<float>();
  const float *pr2 = r2.ptr<float>();

  p11.x = pr1[0];
  p11.y = pr1[1];

  p22.x = pr2[0];
  p22.y = pr2[1];

  cv::Mat x3D;
  cv::Mat Tcw1 = (cv::Mat_<float>(3, 4) << 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f);
  cv::Mat Tcw2;
  cv::Mat R21 = R12.t();
  cv::Mat t21 = -R21 * t12;
  cv::hconcat(R21, t21, Tcw2);

  Triangulate(p11, p22, Tcw1, Tcw2, x3D);
  cv::Mat x3Dt = x3D.t();

  float z1 = x3D.at<float>(2);
  if (z1 <= 0) {
    return -1;
  }

  float z2 = R21.row(2).dot(x3Dt) + t21.at<float>(2);
  if (z2 <= 0) {
    return -1;
  }

  // Check reprojection error
  cv::Point2f uv1 = this->project(x3D);

  float errX1 = uv1.x - kp1.pt.x;
  float errY1 = uv1.y - kp1.pt.y;

  if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel) {  // Reprojection error is high
    return -1;
  }

  cv::Mat x3D2 = R21 * x3D + t21;
  cv::Point2f uv2 = pCamera2->project(x3D2);

  float errX2 = uv2.x - kp2.pt.x;
  float errY2 = uv2.y - kp2.pt.y;

  if ((errX2 * errX2 + errY2 * errY2) > 5.991 * unc) {  // Reprojection error is high
    return -1;
  }

  p3D = x3D.clone();

  if (pz2) *pz2 = z2;
  return z1;
}

// float GeometricCamera::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) { return 1.0; }

cv::Mat GeometricCamera::toKcv() {
  return Converter::toCvMat(toK());
}
bool GeometricCamera::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                const cv::Mat &R12in, const cv::Mat &t12in, const float sigmaLevel, const float unc) {
  // Compute Fundamental Matrix
  Eigen::Vector3d t12 = Converter::toVector3d(t12in);
  Eigen::Matrix3d R12 = Converter::toMatrix3d(R12in);
  auto t12x = Sophus::SO3exd::hat(t12);
  auto K1 = this->toK();
  auto K2 = pCamera2->toK();
  Eigen::Matrix3d F12 = K1.transpose().inverse() * t12x * R12 * K2.inverse();

  // Epipolar line in second image l = x1'F12 = [a b c]
  auto pt1 = unproject(kp1.pt), pt2 = unproject(kp2.pt);
  const float a = pt1.x * F12(0, 0) + pt1.y * F12(1, 0) + F12(2, 0);
  const float b = pt1.x * F12(0, 1) + pt1.y * F12(1, 1) + F12(2, 1);
  const float c = pt1.x * F12(0, 2) + pt1.y * F12(1, 2) + F12(2, 2);

  const float num = a * pt2.x + b * pt2.y + c;

  const float den = a * a + b * b;

  if (den == 0) return false;

  const float dsqr = num * num / den;

  return dsqr < 3.84 * unc;
}

//    bool GeometricCamera::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>&
//    vKeys2, const std::vector<int> &vMatches12,
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