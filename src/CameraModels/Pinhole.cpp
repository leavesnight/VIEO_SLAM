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
#include "Converter.h"

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

namespace VIEO_SLAM {

long unsigned int GeometricCamera::nNextId = 0;

Pinhole::Pinhole(cv::FileStorage &fSettings, int id, bool &bmiss_param) {
  string cam_name = "Camera" + (!id ? "" : to_string(id + 1));

  cv::FileNode node_tmp = fSettings[cam_name + ".fx"];
  if (node_tmp.empty()) {
    bmiss_param = true;
    return;
  }
  float fx = node_tmp;
  node_tmp = fSettings[cam_name + ".fy"];
  if (node_tmp.empty()) {
    bmiss_param = true;
    return;
  }
  float fy = fSettings[cam_name + ".fy"];
  node_tmp = fSettings[cam_name + ".cx"];
  if (node_tmp.empty()) {
    bmiss_param = true;
    return;
  }
  float cx = fSettings[cam_name + ".cx"];
  node_tmp = fSettings[cam_name + ".cy"];
  if (node_tmp.empty()) {
    bmiss_param = true;
    return;
  }
  float cy = fSettings[cam_name + ".cy"];

  mvParameters.resize(4);
  mvParameters[0] = fx;
  mvParameters[1] = fy;
  mvParameters[2] = cx;
  mvParameters[3] = cy;

  PRINT_INFO_MUTEX(endl << "Camera (Pinhole) Parameters: " << endl);
  PRINT_INFO_MUTEX("- fx: " << fx << endl);
  PRINT_INFO_MUTEX("- fy: " << fy << endl);
  PRINT_INFO_MUTEX("- cx: " << cx << endl);
  PRINT_INFO_MUTEX("- cy: " << cy << endl);

  node_tmp = fSettings[cam_name + ".Trc"];
  cv::Mat &Trc = Trc_;
  Eigen::Matrix3d &Rcr = Rcr_;
  Eigen::Vector3d &tcr = tcr_;
  if (!node_tmp.empty()) {
    Trc = node_tmp.mat();
    if (Trc.rows != 3 || Trc.cols != 4) {
      std::cerr << "*Trc matrix have to be a 3x4 transformation matrix*" << std::endl;
      bmiss_param = true;
      return;
    }
    Rcr = Converter::toMatrix3d(Trc.rowRange(0, 3).colRange(0, 3)).transpose();
    tcr = -Rcr * Converter::toVector3d(Trc.col(3));
  } else {
    PRINT_INFO_MUTEX("Warning:*Trc matrix doesn't exist*" << std::endl);
    Trc = cv::Mat::eye(3, 4, CV_32F);
  }
  PRINT_INFO_MUTEX("- Trc: \n" << Trc << std::endl);

  bmiss_param = false;
}

bool Pinhole::ParseCamParamFile(cv::FileStorage &fSettings, int id, GeometricCamera *&pCamInst, cv::Mat *pK,
                                cv::Mat *pDistCoef) {
  bool b_miss_params = false;
  pCamInst = new Pinhole(fSettings, id, b_miss_params);
  if (b_miss_params) {
    cerr << "Error: miss params!" << endl;
    return false;
  }
  if (pK) pCamInst->toKcv().copyTo(*pK);
  if (pDistCoef) *pDistCoef = cv::Mat::zeros(4, 1, CV_32F);

  return true;
}

Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &p3D) {
  const double invz = 1. / p3D[2];
  auto pt = Eigen::Vector2d(mvParameters[0] * p3D[0] * invz + mvParameters[2],
                            mvParameters[1] * p3D[1] * invz + mvParameters[3]);
  return pt;
}

// float Pinhole::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) { return 1.0; }

Eigen::Vector3d Pinhole::unproject(const Eigen::Vector2d &p2D) {
  return Eigen::Vector3d((p2D[0] - mvParameters[2]) / mvParameters[0], (p2D[1] - mvParameters[3]) / mvParameters[1],
                         1.);
}

Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
  Eigen::Matrix<double, 2, 3> jac;
  double invz = 1 / v3D[2];
  CV_Assert(4 == mvParameters.size());
  double invz_2 = invz * invz;
  jac << mvParameters[0] * invz, 0, -v3D[0] * mvParameters[0] * invz_2, 0, mvParameters[1] * invz,
      -v3D[1] * mvParameters[1] * invz_2;

  return jac;
}

Eigen::Matrix3d Pinhole::toK() {
  Eigen::Matrix3d K;
  K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f;
  return K;
}
}  // namespace VIEO_SLAM
