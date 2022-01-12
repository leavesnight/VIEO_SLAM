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

#include "common/log.h"
#include "eigen_utils.h"
#include "sophus/se3.hpp"

//#include "TwoViewReconstruction.h"

namespace VIEO_SLAM {
class GeometricCamera {
 public:
  template <typename _Tp>
  using vector = std::vector<_Tp>;
  template <typename _Tp>
  using aligned_vector = Eigen::aligned_vector<_Tp>;
  using Matrix34 = Eigen::Matrix<double, 3, 4>;

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

  virtual vector<float> TriangulateMatches(const vector<GeometricCamera*>& pcams_in,
                                           const aligned_vector<Eigen::Vector2d>& kps, const vector<float>& sigmaLevels,
                                           cv::Mat* p3D = nullptr, double thresh_cosdisparity = 0.9998,
                                           const vector<vector<float>>* pur = nullptr,
                                           const aligned_vector<Sophus::SE3d>* pTwr = nullptr,
                                           bool just_check_p3d = false);

  // virtual float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D);

  // Suppose GeometricCamera is a P2D = K * normedP3D or linear mapping Camera Model,
  // if it's nonlinear, please impelment the following virtual funcs
  virtual Eigen::Matrix3d toK() = 0;
  virtual cv::Mat toKcv();
  virtual bool epipolarConstrain(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                 const cv::Mat& R12, const cv::Mat& t12, const float sigmaLevel, const float unc);

  virtual bool FillMatchesFromPair(const vector<GeometricCamera*>& pcams, size_t n_cams_tot,
                                   const vector<std::pair<size_t, size_t>>& vcamidx, double dist,
                                   vector<vector<size_t>>& mvidxsMatches, vector<bool>& goodmatches_,
                                   std::map<std::pair<size_t, size_t>, size_t>& mapcamidx2idxs_,
                                   const double thresh_cosdisparity = 1. - 1.e-6,
                                   aligned_vector<Eigen::Vector3d>* pv3dpoints = nullptr,
                                   aligned_vector<Eigen::Vector2d>* pkpts = nullptr, vector<float>* psigmas = nullptr,
                                   vector<vector<double>>* plastdists = nullptr, int* pcount_descmatch = nullptr);

  // for monocular init
  //  virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>&
  //  vKeys2,
  //                                       const std::vector<int>& vMatches12, cv::Mat& R21, cv::Mat& t21,
  //                                       std::vector<cv::Point3f>& vP3D, std::vector<bool>& vbTriangulated);

  size_t size() { return mvParameters.size(); }

  unsigned int GetType() { return mnType; }

  enum CAM_TYPE { CAM_PINHOLE, CAM_FISHEYE, CAM_RADTAN };

  static long unsigned int nNextId;

  const Sophus::SE3d& GetTrc() { return Trc_; }
  const Sophus::SE3d& GetTcr() { return Tcr_; }
  void SetTrc(const Sophus::SE3d& Trc) {
    Trc_ = Trc;
    Tcr_ = Trc.inverse();
  }
  const cv::Mat Getcvtrc();

 protected:
  std::vector<float> mvParameters;

  unsigned int mnId;

  unsigned int mnType;

  Sophus::SE3d Trc_ = Sophus::SE3d(), Tcr_ = Sophus::SE3d();  // ref->cam, e.g. Tlr for 2cams
  //        TwoViewReconstruction* tvr;

  bool Triangulate(const aligned_vector<Eigen::Vector2d>& ps, const aligned_vector<Matrix34>& Tcws,
                   Eigen::Vector3d& x3D);
};
}  // namespace VIEO_SLAM

#endif  // CAMERAMODELS_GEOMETRICCAMERA_H
