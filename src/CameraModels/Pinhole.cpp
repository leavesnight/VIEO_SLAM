/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of
 * Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Pinhole.h"

#include <boost/serialization/export.hpp>
#include <iostream>
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

namespace VIEO_SLAM {

long unsigned int GeometricCamera::nNextId = 0;

bool Pinhole::ParseCamParamFile(cv::FileStorage &fSettings, int id, GeometricCamera *&pCamInst, cv::Mat *pK,
                                cv::Mat *pDistCoef) {
  string cam_name = "Camera" + (!id ? "" : to_string(id + 1));
  cv::FileNode node_tmp = fSettings[cam_name + ".fx"];
  if (node_tmp.empty()) return false;
  float fx = fSettings[cam_name + ".fx"];
  float fy = fSettings[cam_name + ".fy"];
  float cx = fSettings[cam_name + ".cx"];
  float cy = fSettings[cam_name + ".cy"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  if (pK) K.copyTo(*pK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings[cam_name + ".k1"];
  DistCoef.at<float>(1) = fSettings[cam_name + ".k2"];
  DistCoef.at<float>(2) = fSettings[cam_name + ".p1"];
  DistCoef.at<float>(3) = fSettings[cam_name + ".p2"];
  const float k3 = fSettings[cam_name + ".k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  if (pDistCoef) DistCoef.copyTo(*pDistCoef);

  vector<float> vCamCalib{fx, fy, cx, cy};

  pCamInst = new Pinhole(vCamCalib);

  cout << endl << "Camera (Pinhole) Parameters: " << endl;
  cout << "- fx: " << fx << endl;
  cout << "- fy: " << fy << endl;
  cout << "- cx: " << cx << endl;
  cout << "- cy: " << cy << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if (DistCoef.rows == 5) cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;

  // TODO: check the input
  return true;
}

cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
  return cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
                     mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
}

cv::Point2f Pinhole::project(const cv::Matx31f &m3D) { return this->project(cv::Point3f(m3D(0), m3D(1), m3D(2))); }

cv::Point2f Pinhole::project(const cv::Mat &m3D) {
  const float *p3D = m3D.ptr<float>();

  return this->project(cv::Point3f(p3D[0], p3D[1], p3D[2]));
}

Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
  Eigen::Vector2d res;
  const double invz = 1.0f / v3D[2];  // normalize
  res[0] = mvParameters[0] * v3D[0] * invz + mvParameters[2];
  res[1] = mvParameters[1] * v3D[1] * invz + mvParameters[3];

  return res;
}

cv::Mat Pinhole::projectMat(const cv::Point3f &p3D) {
  cv::Point2f point = this->project(p3D);
  return (cv::Mat_<float>(2, 1) << point.x, point.y);
}

float Pinhole::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) { return 1.0; }

cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
  return cv::Point3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1], 1.f);
}

cv::Mat Pinhole::unprojectMat(const cv::Point2f &p2D) {
  cv::Point3f ray = this->unproject(p2D);
  return (cv::Mat_<float>(3, 1) << ray.x, ray.y, ray.z);
}

cv::Matx31f Pinhole::unprojectMat_(const cv::Point2f &p2D) {
  cv::Point3f ray = this->unproject(p2D);
  cv::Matx31f r{ray.x, ray.y, ray.z};
  return r;
}

cv::Mat Pinhole::projectJac(const cv::Point3f &p3D) {
  cv::Mat Jac(2, 3, CV_32F);
  Jac.at<float>(0, 0) = mvParameters[0] / p3D.z;
  Jac.at<float>(0, 1) = 0.f;
  Jac.at<float>(0, 2) = -mvParameters[0] * p3D.x / (p3D.z * p3D.z);
  Jac.at<float>(1, 0) = 0.f;
  Jac.at<float>(1, 1) = mvParameters[1] / p3D.z;
  Jac.at<float>(1, 2) = -mvParameters[1] * p3D.y / (p3D.z * p3D.z);

  return Jac;
}

Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
  Eigen::Matrix<double, 2, 3> jac;
  double invz = 1 / v3D[2], invz_2 = invz * invz;
  jac << mvParameters[0] * invz, 0, -v3D[0] * mvParameters[0] * invz_2, 0, mvParameters[1] * invz,
      -v3D[1] * mvParameters[1] * invz_2;

  return jac;
}

cv::Mat Pinhole::unprojectJac(const cv::Point2f &p2D) {
  cv::Mat Jac(3, 2, CV_32F);
  Jac.at<float>(0, 0) = 1 / mvParameters[0];
  Jac.at<float>(0, 1) = 0.f;
  Jac.at<float>(1, 0) = 0.f;
  Jac.at<float>(1, 1) = 1 / mvParameters[1];
  Jac.at<float>(2, 0) = 0.f;
  Jac.at<float>(2, 1) = 0.f;

  return Jac;
}

//    bool Pinhole::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>&
//    vKeys2, const std::vector<int> &vMatches12,
//                                 cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool>
//                                 &vbTriangulated){
//        if(!tvr){
//            cv::Mat K = this->toK();
//            tvr = new TwoViewReconstruction(K);
//        }
//
//        return tvr->Reconstruct(vKeys1,vKeys2,vMatches12,R21,t21,vP3D,vbTriangulated);
//    }

cv::Mat Pinhole::toK() {
  cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3],
               0.f, 0.f, 1.f);
  return K;
}

cv::Matx33f Pinhole::toK_() {
  cv::Matx33f K{mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f};

  return K;
}

bool Pinhole::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc) {
  // Compute Fundamental Matrix
  cv::Mat t12x = SkewSymmetricMatrix(t12);
  cv::Mat K1 = this->toK();
  cv::Mat K2 = pCamera2->toK();
  cv::Mat F12 = K1.t().inv() * t12x * R12 * K2.inv();

  // Epipolar line in second image l = x1'F12 = [a b c]
  const float a = kp1.pt.x * F12.at<float>(0, 0) + kp1.pt.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
  const float b = kp1.pt.x * F12.at<float>(0, 1) + kp1.pt.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
  const float c = kp1.pt.x * F12.at<float>(0, 2) + kp1.pt.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

  const float num = a * kp2.pt.x + b * kp2.pt.y + c;

  const float den = a * a + b * b;

  if (den == 0) return false;

  const float dsqr = num * num / den;

  return dsqr < 3.84 * unc;
}

bool Pinhole::epipolarConstrain_(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                 const cv::Matx33f &R12, const cv::Matx31f &t12, const float sigmaLevel,
                                 const float unc) {
  // Compute Fundamental Matrix
  auto t12x = SkewSymmetricMatrix_(t12);
  auto K1 = this->toK_();
  auto K2 = pCamera2->toK_();
  cv::Matx33f F12 = K1.t().inv() * t12x * R12 * K2.inv();

  // Epipolar line in second image l = x1'F12 = [a b c]
  const float a = kp1.pt.x * F12(0, 0) + kp1.pt.y * F12(1, 0) + F12(2, 0);
  const float b = kp1.pt.x * F12(0, 1) + kp1.pt.y * F12(1, 1) + F12(2, 1);
  const float c = kp1.pt.x * F12(0, 2) + kp1.pt.y * F12(1, 2) + F12(2, 2);

  const float num = a * kp2.pt.x + b * kp2.pt.y + c;

  const float den = a * a + b * b;

  if (den == 0) return false;

  const float dsqr = num * num / den;

  return dsqr < 3.84 * unc;
}

std::ostream &operator<<(std::ostream &os, const Pinhole &ph) {
  os << ph.mvParameters[0] << " " << ph.mvParameters[1] << " " << ph.mvParameters[2] << " " << ph.mvParameters[3];
  return os;
}

std::istream &operator>>(std::istream &is, Pinhole &ph) {
  float nextParam;
  for (size_t i = 0; i < 4; i++) {
    assert(is.good());  // Make sure the input stream is good
    is >> nextParam;
    ph.mvParameters[i] = nextParam;
  }
  return is;
}

cv::Mat Pinhole::SkewSymmetricMatrix(const cv::Mat &v) {
  return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1), v.at<float>(2), 0, -v.at<float>(0),
          -v.at<float>(1), v.at<float>(0), 0);
}

cv::Matx33f Pinhole::SkewSymmetricMatrix_(const cv::Matx31f &v) {
  cv::Matx33f skew{0.f, -v(2), v(1), v(2), 0.f, -v(0), -v(1), v(0), 0.f};

  return skew;
}
}  // namespace VIEO_SLAM
