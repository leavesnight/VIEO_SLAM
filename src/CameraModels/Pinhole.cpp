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
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
using std::cerr;
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
  bool b_miss_params = false;

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

  pCamInst = new Pinhole(DistCoef, fSettings, id, b_miss_params);
  if (b_miss_params) {
    cerr << "Error: miss params!" << endl;
    return false;
  }
  if (pK) pCamInst->toK().copyTo(*pK);

  cout << endl << "Camera (Pinhole) Parameters: " << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if (DistCoef.rows == 5) cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;

  // TODO: check the input
  return true;
}

Eigen::Vector2d Pinhole::distortPoints(float x, float y) {
  Eigen::Vector2d pt;
  if (mvParameters.size() >= 8) {
    double x2 = x * x, y2 = y * y, r2 = x2 + y2, r4 = r2 * r2, xy = x * y;  //,r6=r2*r4;
    float *k = mvParameters.data() + 4, *p = k + 2;
    double fd = 1 + k[0] * r2 + k[1] * r4;
    if (mvParameters.size() > 8) {
      double term_r = r4;
      for (int i = 2; i < mvParameters.size() - 6; ++i) {
        term_r *= r2;
        fd += k[i] * term_r;
      }
    }
    pt[0] = x * fd + 2 * p[0] * xy + p[1] * (r2 + 2 * x2);
    pt[1] = y * fd + 2 * p[1] * xy + p[0] * (r2 + 2 * y2);
  } else {
    pt[0] = x;
    pt[1] = y;
  }
  pt[0] = mvParameters[0] * pt[0] + mvParameters[2];
  pt[1] = mvParameters[1] * pt[1] + mvParameters[3];
  return pt;
}

// TODO: pinhole project with no distort becomes GeometricCamera
cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
  //  auto pt = cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
  //                     mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
  cv::Point2f ptout;
  auto pt = distortPoints(p3D.x / p3D.z, p3D.y / p3D.z);
  ptout.x = pt[0];
  ptout.y = pt[1];
  return ptout;
}

cv::Point2f Pinhole::project(const cv::Matx31f &m3D) { return this->project(cv::Point3f(m3D(0), m3D(1), m3D(2))); }

cv::Point2f Pinhole::project(const cv::Mat &m3D) {
  const float *p3D = m3D.ptr<float>();

  return this->project(cv::Point3f(p3D[0], p3D[1], p3D[2]));
}

Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
  const double invz = 1.0f / v3D[2];  // normalize
  return distortPoints(v3D[0] * invz, v3D[1] * invz);
}
cv::Mat Pinhole::toDistortCoeff() {
  if (mvParameters.size() == 8)
    return (cv::Mat_<float>(4, 1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
  else
    return cv::Mat::zeros(4, 1, CV_32F);
}

cv::Mat Pinhole::projectMat(const cv::Point3f &p3D) {
  cv::Point2f point = this->project(p3D);
  return (cv::Mat_<float>(2, 1) << point.x, point.y);
}

float Pinhole::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) { return 1.0; }

cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
  cv::Mat pt = (cv::Mat_<float>(1, 2) << p2D.x, p2D.y);
  auto K = toK();
  pt.reshape(2);
  // final no K means undistort to normalized plane
  cv::undistortPoints(pt, pt, K, toDistortCoeff(), cv::Mat());
  pt.reshape(1);

  return cv::Point3f(pt.at<float>(0), pt.at<float>(1), 1);
  //  return cv::Point3f((pt.at<float>(0) - mvParameters[2]) / mvParameters[0],
  //                     (pt.at<float>(1) - mvParameters[3]) / mvParameters[1], 1.f);
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
  Eigen::Matrix<double, 2, 3> jac = projectJac(Eigen::Vector3d(p3D.x, p3D.y, p3D.z));
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 3; ++j) Jac.at<float>(i, j) = jac(i, j);
  return Jac;
}

Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
  Eigen::Matrix<double, 2, 3> jac;
  double invz = 1 / v3D[2];
  if (mvParameters.size() == 8) {
    double x = v3D[0] * invz, y = v3D[1] * invz;
    double x2 = x * x, y2 = y * y, r2 = x2 + y2, r4 = r2 * r2, xy = x * y;  //,r6=r2*r4;
    float *k = mvParameters.data() + 4, *p = k + mvParameters.size() - 6;
    double fd = 1 + k[0] * r2 + k[1] * r4, fd2 = 2 * k[0] + 4 * k[1] * r2;
    if (mvParameters.size() > 8) {
      double term_r = r4, coeff2 = 4;
      for (int i = 2; i < mvParameters.size() - 6; ++i) {
        coeff2 += 2;
        fd2 += k[i] * coeff2 * term_r;
        term_r *= r2;
        fd += k[i] * term_r;
      }
    }
    jac(0, 0) = mvParameters[0] * invz * (fd + fd2 * x2 + 2 * (p[0] * y + 3 * p[1] * x));
    jac(0, 1) = mvParameters[0] * invz * (fd2 * xy + 2 * (p[0] * x + p[1] * y));
    jac(0, 2) = -(x * jac(0, 0) + y * jac(0, 1));
    jac(1, 0) = jac(0, 1) * mvParameters[1] / mvParameters[0];
    jac(1, 1) = mvParameters[1] * invz * (fd + fd2 * y2 + 2 * (p[1] * x + 3 * p[0] * y));
    jac(1, 2) = -(x * jac(1, 0) + y * jac(1, 1));
  } else {
    double invz_2 = invz * invz;
    jac << mvParameters[0] * invz, 0, -v3D[0] * mvParameters[0] * invz_2, 0, mvParameters[1] * invz,
        -v3D[1] * mvParameters[1] * invz_2;
  }

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
  // TODO: for radtan model
  CV_Assert(0);

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

bool Pinhole::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc) {
  // Compute Fundamental Matrix
  cv::Mat t12x = SkewSymmetricMatrix(t12);
  cv::Mat K1 = this->toK();
  cv::Mat K2 = pCamera2->toK();
  cv::Mat F12 = K1.t().inv() * t12x * R12 * K2.inv();

  // Epipolar line in second image l = x1'F12 = [a b c]
  auto pt1 = unproject(kp1.pt), pt2 = unproject(kp2.pt);
  const float a = pt1.x * F12.at<float>(0, 0) + pt1.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
  const float b = pt1.x * F12.at<float>(0, 1) + pt1.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
  const float c = pt1.x * F12.at<float>(0, 2) + pt1.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

  const float num = a * pt2.x + b * pt2.y + c;

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
