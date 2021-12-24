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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <Eigen/Geometry>
#include "common/log.h"

namespace VIEO_SLAM {
class GeometricCamera {
 public:
  GeometricCamera() {}
  GeometricCamera(const std::vector<float>& _vParameters) : mvParameters(_vParameters) {}
  virtual ~GeometricCamera() {}

  void setParameter(const float p, const size_t i) { mvParameters[i] = p; }

  virtual Eigen::Vector2d project(const Eigen::Vector3d& v3D) = 0;
  virtual cv::Point2f project(const cv::Point3f& p3D);
  virtual cv::Point2f project(const cv::Mat& m3D);

  virtual Eigen::Vector3d unproject(const Eigen::Vector2d& p2D) = 0;
  virtual cv::Point3f unproject(const cv::Point2f& p2D);
  virtual cv::Mat unprojectMat(const cv::Point2f& p2D);

  virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) = 0;
  // virtual cv::Mat projectJac(const cv::Point3f& p3D);

  virtual float TriangulateMatches(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                   const float sigmaLevel, const float unc, cv::Mat& p3D, float* pz2 = nullptr);

  // virtual float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D);

  // Suppose GeometricCamera is a P2D = K * normedP3D or linear mapping Camera Model,
  // if it's nonlinear, please impelment the following virtual funcs
  virtual Eigen::Matrix3d toK() = 0;
  virtual cv::Mat toKcv();
  virtual bool epipolarConstrain(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                 const cv::Mat& R12, const cv::Mat& t12, const float sigmaLevel, const float unc);
  // for monocular init
  //  virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>&
  //  vKeys2,
  //                                       const std::vector<int>& vMatches12, cv::Mat& R21, cv::Mat& t21,
  //                                       std::vector<cv::Point3f>& vP3D, std::vector<bool>& vbTriangulated);

  size_t size() { return mvParameters.size(); }

  unsigned int GetType() { return mnType; }

  enum CAM_TYPE { CAM_PINHOLE, CAM_FISHEYE, CAM_RADTAN };

  static long unsigned int nNextId;

  cv::Mat Trc_;  // ref->cam, Tlr for 2cams
  Eigen::Matrix3d Rcr_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d tcr_ = Eigen::Vector3d::Zero();

 protected:
  std::vector<float> mvParameters;

  unsigned int mnId;

  unsigned int mnType;

  //        TwoViewReconstruction* tvr;

  void Triangulate(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Mat& Tcw1, const cv::Mat& Tcw2,
                   cv::Mat& x3D);
};
}  // namespace VIEO_SLAM

#endif  // CAMERAMODELS_GEOMETRICCAMERA_H
