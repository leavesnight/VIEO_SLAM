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

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include "GeometricCamera.h"

namespace VIEO_SLAM {
class Pinhole : public GeometricCamera {
 public:
  Pinhole() {
    mvParameters.resize(4);
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }
  Pinhole(const vector<float> &params) : GeometricCamera(params) {
    mnId = nNextId++;
    mnType = CAM_PINHOLE;
  }
  Pinhole(cv::FileStorage& fSettings, int id, bool& bmiss_param);
  ~Pinhole() {}

  static bool ParseCamParamFile(cv::FileStorage& fSettings, int id, GeometricCamera*& pCameraInstance, cv::Mat* pK,
                                cv::Mat* pDistCoef);

  Eigen::Vector2d project(const Eigen::Vector3d& p3D) override;

  Eigen::Vector3d unproject(const Eigen::Vector2d& p2D) override;

  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) override;

  Eigen::Matrix3d toK() override;
};
}  // namespace VIEO_SLAM

#endif  // CAMERAMODELS_PINHOLE_H
