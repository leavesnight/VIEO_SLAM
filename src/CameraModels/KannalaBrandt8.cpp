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

#include "KannalaBrandt8.h"

#include <boost/serialization/export.hpp>
#include <iostream>
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

namespace VIEO_SLAM {

bool KannalaBrandt8::ParseCamParamFile(cv::FileStorage &fSettings, int id, GeometricCamera *&pCamInst, cv::Mat *pK,
                                       cv::Mat *pDistCoef) {
  string cam_name = "Camera" + (!id ? "" : to_string(id + 1));
  cv::FileNode node_tmp = fSettings[cam_name + ".k1"];
  if (node_tmp.empty()) return false;

  bool b_miss_params = false;

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings[cam_name + ".k1"];
  DistCoef.at<float>(1) = fSettings[cam_name + ".k2"];
  DistCoef.at<float>(2) = fSettings[cam_name + ".k3"];
  DistCoef.at<float>(3) = fSettings[cam_name + ".k4"];
  if (pDistCoef) DistCoef.copyTo(*pDistCoef);

  pCamInst = new KannalaBrandt8(DistCoef, fSettings, id, b_miss_params);
  if (b_miss_params) return false;
  if (pK) pCamInst->toK().copyTo(*pK);

  PRINT_INFO_MUTEX( endl << cam_name << " (KB8) Parameters: " << endl);
  PRINT_INFO_MUTEX( "- k1: " << DistCoef.at<float>(0) << endl);
  PRINT_INFO_MUTEX( "- k2: " << DistCoef.at<float>(1) << endl);
  PRINT_INFO_MUTEX( "- k3: " << DistCoef.at<float>(2) << endl);
  PRINT_INFO_MUTEX( "- k4: " << DistCoef.at<float>(3) << endl);

  int LappingBegin = -1;
  int LappingEnd = -1;

  cv::FileNode node = fSettings[cam_name + ".lappingBegin"];
  if (!node.empty() && node.isInt())
    LappingBegin = node.operator int();
  else
    PRINT_INFO_MUTEX( "WARNING: Camera.lappingBegin not correctly defined" << std::endl);
  node = fSettings[cam_name + ".lappingEnd"];
  if (!node.empty() && node.isInt())
    LappingEnd = node.operator int();
  else
    PRINT_INFO_MUTEX( "WARNING: Camera.lappingEnd not correctly defined" << std::endl);

  if (!b_miss_params) {
    static_cast<KannalaBrandt8 *>(pCamInst)->mvLappingArea[0] = LappingBegin;
    static_cast<KannalaBrandt8 *>(pCamInst)->mvLappingArea[1] = LappingEnd;

    // mpFrameDrawer->both = true;

    PRINT_INFO_MUTEX( "- " << cam_name << " Lapping: " << LappingBegin << ", " << LappingEnd << std::endl);
  }

  // TODO: check the input
  return true;
}

cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D) {
  const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
  const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
  const float psi = atan2f(p3D.y, p3D.x);

  const float theta2 = theta * theta;
  const float theta3 = theta * theta2;
  const float theta5 = theta3 * theta2;
  const float theta7 = theta5 * theta2;
  const float theta9 = theta7 * theta2;
  const float r =
      theta + mvParameters[4] * theta3 + mvParameters[5] * theta5 + mvParameters[6] * theta7 + mvParameters[7] * theta9;

  return cv::Point2f(mvParameters[0] * r * cos(psi) + mvParameters[2],
                     mvParameters[1] * r * sin(psi) + mvParameters[3]);
}

cv::Point2f KannalaBrandt8::project(const cv::Matx31f &m3D) {
  return this->project(cv::Point3f(m3D(0), m3D(1), m3D(2)));
}

cv::Point2f KannalaBrandt8::project(const cv::Mat &m3D) {
  const float *p3D = m3D.ptr<float>();

  return this->project(cv::Point3f(p3D[0], p3D[1], p3D[2]));
}

Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D) {
  const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
  double r = sqrtf(x2_plus_y2);
  const double theta = atan2(sqrtf(x2_plus_y2), v3D[2]);  // notice theta could be [pi/2,pi]
  const double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta4 * theta4;
  const double thetad = theta * (1 + mvParameters[4] * theta2 + mvParameters[5] * theta4 + mvParameters[6] * theta6 +
                                 mvParameters[7] * theta8);

  Eigen::Vector2d res;
  res[0] = thetad / r * v3D[0];  // orb3 uses cos(psi), check whose efficiency is better
  res[1] = thetad / r * v3D[1];
  res[0] = res[0] * mvParameters[0] + mvParameters[2];
  res[1] = res[1] * mvParameters[1] + mvParameters[3];
  /*const double psi = atan2f(v3D[1], v3D[0]);
  res[0] = mvParameters[0] * thetad * cos(psi) + mvParameters[2];
  res[1] = mvParameters[1] * thetad * sin(psi) + mvParameters[3];*/

  return res;
}

cv::Mat KannalaBrandt8::projectMat(const cv::Point3f &p3D) {
  cv::Point2f point = this->project(p3D);
  cv::Mat ret = (cv::Mat_<float>(2, 1) << point.x, point.y);
  return ret.clone();
}

float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) { return 1.f; }

cv::Mat KannalaBrandt8::unprojectMat(const cv::Point2f &p2D) {
  cv::Point3f ray = this->unproject(p2D);
  cv::Mat ret = (cv::Mat_<float>(3, 1) << ray.x, ray.y, ray.z);
  return ret.clone();
}

cv::Matx31f KannalaBrandt8::unprojectMat_(const cv::Point2f &p2D) {
  cv::Point3f ray = this->unproject(p2D);
  cv::Matx31f r{ray.x, ray.y, ray.z};
  return r;
}

cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D) {
  // Use Newthon method to solve for theta with good precision (err ~ e-6)
  cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
  float scale = 1.f;
  float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
  theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

  if (theta_d > 1e-8) {
    // Compensate distortion iteratively
    float theta = theta_d;

    for (int j = 0; j < 10; j++) {
      float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta4 * theta4;
      float k0_theta2 = mvParameters[4] * theta2, k1_theta4 = mvParameters[5] * theta4;
      float k2_theta6 = mvParameters[6] * theta6, k3_theta8 = mvParameters[7] * theta8;
      float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                        (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
      theta = theta - theta_fix;
      if (fabsf(theta_fix) < precision) break;
    }
    // scale = theta - theta_d;
    scale = std::tan(theta) / theta_d;
  }

  return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
}

cv::Mat KannalaBrandt8::projectJac(const cv::Point3f &p3D) {
  float x2 = p3D.x * p3D.x, y2 = p3D.y * p3D.y, z2 = p3D.z * p3D.z;
  float r2 = x2 + y2;
  float r = sqrt(r2);
  float r3 = r2 * r;
  float theta = atan2(r, p3D.z);

  float theta2 = theta * theta, theta3 = theta2 * theta;
  float theta4 = theta2 * theta2, theta5 = theta4 * theta;
  float theta6 = theta2 * theta4, theta7 = theta6 * theta;
  float theta8 = theta4 * theta4, theta9 = theta8 * theta;

  float f =
      theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] + theta9 * mvParameters[7];
  float fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
             9 * mvParameters[7] * theta8;

  cv::Mat Jac(2, 3, CV_32F);
  Jac.at<float>(0, 0) = mvParameters[0] * (fd * p3D.z * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
  Jac.at<float>(1, 0) = mvParameters[1] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);

  Jac.at<float>(0, 1) = mvParameters[0] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);
  Jac.at<float>(1, 1) = mvParameters[1] * (fd * p3D.z * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

  Jac.at<float>(0, 2) = -mvParameters[0] * fd * p3D.x / (r2 + z2);
  Jac.at<float>(1, 2) = -mvParameters[1] * fd * p3D.y / (r2 + z2);

  return Jac.clone();
}

Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D) {
  double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
  double r2 = x2 + y2;
  double r = sqrt(r2);
  double r3 = r2 * r;
  double theta = atan2(r, v3D[2]);

  double theta2 = theta * theta, theta3 = theta2 * theta;
  double theta4 = theta2 * theta2, theta5 = theta4 * theta;
  double theta6 = theta2 * theta4, theta7 = theta6 * theta;
  double theta8 = theta4 * theta4, theta9 = theta8 * theta;

  double f =
      theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] + theta9 * mvParameters[7];
  double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
              9 * mvParameters[7] * theta8;

  Eigen::Matrix<double, 2, 3> JacGood;
  JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
  JacGood(1, 0) = mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

  JacGood(0, 1) = mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
  JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

  JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
  JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

  return JacGood;
}

cv::Mat KannalaBrandt8::unprojectJac(const cv::Point2f &p2D) { return cv::Mat(); }

//    bool KannalaBrandt8::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const
//    std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
//                                          cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D,
//                                          std::vector<bool> &vbTriangulated){
//        if(!tvr){
//            cv::Mat K = this->toK();
//            tvr = new TwoViewReconstruction(K);
//        }
//
//        //Correct FishEye distortion
//        std::vector<cv::KeyPoint> vKeysUn1 = vKeys1, vKeysUn2 = vKeys2;
//        std::vector<cv::Point2f> vPts1(vKeys1.size()), vPts2(vKeys2.size());
//
//        for(size_t i = 0; i < vKeys1.size(); i++) vPts1[i] = vKeys1[i].pt;
//        for(size_t i = 0; i < vKeys2.size(); i++) vPts2[i] = vKeys2[i].pt;
//
//        cv::Mat D = (cv::Mat_<float>(4,1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
//        cv::Mat R = cv::Mat::eye(3,3,CV_32F);
//        cv::Mat K = this->toK();
//        cv::fisheye::undistortPoints(vPts1,vPts1,K,D,R,K);
//        cv::fisheye::undistortPoints(vPts2,vPts2,K,D,R,K);
//
//        for(size_t i = 0; i < vKeys1.size(); i++) vKeysUn1[i].pt = vPts1[i];
//        for(size_t i = 0; i < vKeys2.size(); i++) vKeysUn2[i].pt = vPts2[i];
//
//        return tvr->Reconstruct(vKeysUn1,vKeysUn2,vMatches12,R21,t21,vP3D,vbTriangulated);
//    }

bool KannalaBrandt8::epipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                       const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel,
                                       const float unc) {
  cv::Mat p3D;
  return this->TriangulateMatches(pCamera2, kp1, kp2, sigmaLevel, unc, p3D) > 0.0001f;
}
}  // namespace VIEO_SLAM
