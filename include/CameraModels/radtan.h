//
// Created by leavesnight on 2021/12/23.
//

#ifndef VIEO_SLAM_RADTAN_H
#define VIEO_SLAM_RADTAN_H

#include "Pinhole.h"

namespace VIEO_SLAM {
class Radtan : public Pinhole {
 public:
  Radtan() {
    mvParameters.resize(4);
    mnId = nNextId++;
    mnType = CAM_RADTAN;
  }
  Radtan(const cv::Mat& DistCoef, cv::FileStorage& fSettings, int id, bool& bmiss_param)
      : Pinhole(fSettings, id, bmiss_param) {
    CV_Assert((DistCoef.total() == 4 || DistCoef.total() == 5) && DistCoef.elemSize() == sizeof(float));
    mvParameters.resize(4 + DistCoef.total());
    for (int i = 0; i < DistCoef.total(); ++i) {
      mvParameters[4 + i] = DistCoef.at<float>(i);
    }
    mnType = CAM_RADTAN;
  }
  ~Radtan() {}

  static bool ParseCamParamFile(cv::FileStorage& fSettings, int id, GeometricCamera*& pCameraInstance, cv::Mat* pK,
                                cv::Mat* pDistCoef);

  cv::Mat toDistortCoeff();
  Eigen::Vector2d distortPoints(float x, float y);
  Eigen::Vector2d project(const Eigen::Vector3d& v3D) override;

  Eigen::Vector3d unproject(const Eigen::Vector2d& p2D) override;

  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) override;
};
}  // namespace VIEO_SLAM

#endif  // VIEO_SLAM_RADTAN_H
