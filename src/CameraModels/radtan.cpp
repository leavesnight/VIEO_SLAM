//
// Created by leavesnight on 2021/12/23.
//

#include "radtan.h"
#include <opencv2/opencv.hpp>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

namespace VIEO_SLAM {

bool Radtan::ParseCamParamFile(cv::FileStorage &fSettings, int id, GeometricCamera *&pCamInst) {
  string cam_name = "Camera" + (!id ? "" : to_string(id + 1));
  cv::FileNode node_tmp = fSettings[cam_name + ".fx"];
  if (node_tmp.empty()) return false;
  bool b_miss_params = false;

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings[cam_name + ".k1"];
  DistCoef.at<float>(1) = fSettings[cam_name + ".k2"];
  int id_distcoef = 2;
  node_tmp = fSettings[cam_name + ".k3"];
  if (!node_tmp.empty()) {
    const float k3 = (float)node_tmp;
    if (k3 != 0) {
      DistCoef.resize(5);
      DistCoef.at<float>(id_distcoef++) = k3;
    }
  }
  DistCoef.at<float>(id_distcoef++) = fSettings[cam_name + ".p1"];
  DistCoef.at<float>(id_distcoef++) = fSettings[cam_name + ".p2"];

  pCamInst = new Radtan(DistCoef, fSettings, id, b_miss_params);
  if (b_miss_params) {
    cerr << "Error: miss params!" << endl;
    return false;
  }

  PRINT_INFO_MUTEX(endl << "Camera (Radtan) Parameters: " << endl);
  PRINT_INFO_MUTEX("- k1: " << DistCoef.at<float>(0) << endl);
  PRINT_INFO_MUTEX("- k2: " << DistCoef.at<float>(1) << endl);
  id_distcoef = 2;
  if (DistCoef.rows == 5) PRINT_INFO_MUTEX("- k3: " << DistCoef.at<float>(id_distcoef++) << endl);
  PRINT_INFO_MUTEX("- p1: " << DistCoef.at<float>(id_distcoef++) << endl);
  PRINT_INFO_MUTEX("- p2: " << DistCoef.at<float>(id_distcoef++) << endl);

  // TODO: check the input
  return true;
}

cv::Mat Radtan::toDistortCoeff_OpenCV() {
  if (mvParameters.size() == 8)
    return (cv::Mat_<float>(4, 1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
  else if (mvParameters.size() == 9)
    return (cv::Mat_<float>(4, 1) << mvParameters[4], mvParameters[5], mvParameters[7], mvParameters[8],
            mvParameters[6]);
  else {
    CV_Assert(0 && "Unimplemented Distort Radtan Model");
    return cv::Mat();
  }
}
Eigen::Vector2d Radtan::distortPoints(float x, float y) {
  Eigen::Vector2d pt;
  if (mvParameters.size() >= 8) {
    double x2 = x * x, y2 = y * y, r2 = x2 + y2, r4 = r2 * r2, xy = x * y;  //,r6=r2*r4;
    float *k = mvParameters.data() + 4, *p = k + mvParameters.size() - 6;
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
  return Pinhole::project(Eigen::Vector3d(pt[0], pt[1], 1.));
}
Eigen::Vector2d Radtan::project(const Eigen::Vector3d &v3D) {
  const double invz = 1.0f / v3D[2];  // normalize
  return distortPoints(v3D[0] * invz, v3D[1] * invz);
}

Eigen::Vector3d Radtan::unproject(const Eigen::Vector2d &p2D) {
  cv::Mat pt(1, 1, CV_32FC2);  // for opencv3.2 cannot reshape 1x1x1 to 1x1x2
  pt.at<cv::Point2f>(0, 0) = cv::Point2f(p2D[0], p2D[1]);
  auto K = toKcv();
  // final no K means undistort to normalized plane
  cv::undistortPoints(pt, pt, K, toDistortCoeff_OpenCV(), cv::Mat(), cv::Mat::eye(3, 3, CV_32F));
  pt.reshape(1);
  return Eigen::Vector3d(pt.at<float>(0), pt.at<float>(1), 1);
}

Eigen::Matrix<double, 2, 3> Radtan::projectJac(const Eigen::Vector3d &v3D) {
  Eigen::Matrix<double, 2, 3> jac;
  double invz = 1 / v3D[2];
  CV_Assert(8 <= mvParameters.size());
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

  return jac;
}
}  // namespace VIEO_SLAM
