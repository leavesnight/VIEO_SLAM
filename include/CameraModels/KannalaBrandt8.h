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

#ifndef CAMERAMODELS_KANNALABRANDT8_H
#define CAMERAMODELS_KANNALABRANDT8_H

#include "Pinhole.h"

namespace VIEO_SLAM {
class KannalaBrandt8 final : public Pinhole {
 public:
  KannalaBrandt8() {
    mvParameters.resize(8);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }
  KannalaBrandt8(const cv::Mat& DistCoef, cv::FileStorage& fSettings, int id, bool& bmiss_param)
      : Pinhole(fSettings, id, bmiss_param) {
    CV_Assert(DistCoef.total() == 4 && DistCoef.elemSize() == sizeof(float));
    mvParameters.resize(8);
    mvParameters[4] = DistCoef.at<float>(0);
    mvParameters[5] = DistCoef.at<float>(1);
    mvParameters[6] = DistCoef.at<float>(2);
    mvParameters[7] = DistCoef.at<float>(3);
    mnId = nNextId++;
    mnType = CAM_FISHEYE;
  }

  static bool ParseCamParamFile(cv::FileStorage& fSettings, int id, GeometricCamera*& pCameraInstance, cv::Mat* pK,
                                cv::Mat* pDistCoef);

  Eigen::Vector2d project(const Eigen::Vector3d& v3D) override;

  Eigen::Vector3d unproject(const Eigen::Vector2d &p2D) override;

  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) override;

  bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                         const cv::Mat& R12, const cv::Mat& t12, const float sigmaLevel, const float unc);

  //  bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2,
  //                               const std::vector<int>& vMatches12, cv::Mat& R21, cv::Mat& t21,
  //                               std::vector<cv::Point3f>& vP3D, std::vector<bool>& vbTriangulated) override;

  std::vector<int> mvLappingArea = {0, INT_MAX};

 private:
  const float precision = 1e-6;
};
}  // namespace VIEO_SLAM

// BOOST_CLASS_EXPORT_KEY(ORBSLAM2::KannalaBrandt8)

#endif  // CAMERAMODELS_KANNALABRANDT8_H
