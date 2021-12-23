//
// Created by leavesnight on 2021/12/7.
//

#include "GeometricCamera.h"
#include <string>
#include <iostream>
#include "Converter.h"
using std::cout;
using std::endl;
using std::string;
using std::to_string;

namespace VIEO_SLAM {

GeometricCamera::GeometricCamera(cv::FileStorage &fSettings, int id, bool &bmiss_param) {
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

  PRINT_INFO_MUTEX( endl << "Camera (Geometric) Parameters: " << endl);
  PRINT_INFO_MUTEX( "- fx: " << fx << endl);
  PRINT_INFO_MUTEX( "- fy: " << fy << endl);
  PRINT_INFO_MUTEX( "- cx: " << cx << endl);
  PRINT_INFO_MUTEX( "- cy: " << cy << endl);

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
    PRINT_INFO_MUTEX( "Warning:*Trc matrix doesn't exist*" << std::endl);
    Trc = cv::Mat::eye(3, 4, CV_32F);
  }
  PRINT_INFO_MUTEX( "- Trc: \n" << Trc << std::endl);

  bmiss_param = false;
}

cv::Mat GeometricCamera::toK() {
  cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3],
               0.f, 0.f, 1.f);
  return K;
}

cv::Matx33f GeometricCamera::toK_() {
  cv::Matx33f K{mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f};

  return K;
}

void GeometricCamera::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Mat &Tcw1,
                                  const cv::Mat &Tcw2, cv::Mat &x3D) {
  cv::Mat A(4, 4, CV_32F);

  A.row(0) = p1.x * Tcw1.row(2) - Tcw1.row(0);
  A.row(1) = p1.y * Tcw1.row(2) - Tcw1.row(1);
  A.row(2) = p2.x * Tcw2.row(2) - Tcw2.row(0);
  A.row(3) = p2.y * Tcw2.row(2) - Tcw2.row(1);

  cv::Mat u, w, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  x3D = vt.row(3).t();
  x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
}

// TODO: change to 4 cams
float GeometricCamera::TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                          const float sigmaLevel, const float unc, cv::Mat &p3D, float *pz2) {
  cv::Mat r1 = this->unprojectMat(kp1.pt);
  cv::Mat r2 = pCamera2->unprojectMat(kp2.pt);

  cv::Mat Rr1T = Trc_.rowRange(0, 3).colRange(0, 3).t();
  cv::Mat R12 = Rr1T * pCamera2->Trc_.rowRange(0, 3).colRange(0, 3);
  cv::Mat t12 = Rr1T * pCamera2->Trc_.col(3) - Rr1T * Trc_.col(3);
  // Check parallax
  cv::Mat r21 = R12 * r2;

  const float cosParallaxRays = r1.dot(r21) / (cv::norm(r1) * cv::norm(r21));

  if (cosParallaxRays > 0.9998) {
    return -1;
  }

  // Parallax is good, so we try to triangulate
  cv::Point2f p11, p22;
  const float *pr1 = r1.ptr<float>();
  const float *pr2 = r2.ptr<float>();

  p11.x = pr1[0];
  p11.y = pr1[1];

  p22.x = pr2[0];
  p22.y = pr2[1];

  cv::Mat x3D;
  cv::Mat Tcw1 = (cv::Mat_<float>(3, 4) << 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f);
  cv::Mat Tcw2;
  cv::Mat R21 = R12.t();
  cv::Mat t21 = -R21 * t12;
  cv::hconcat(R21, t21, Tcw2);

  Triangulate(p11, p22, Tcw1, Tcw2, x3D);
  cv::Mat x3Dt = x3D.t();

  float z1 = x3D.at<float>(2);
  if (z1 <= 0) {
    return -1;
  }

  float z2 = R21.row(2).dot(x3Dt) + t21.at<float>(2);
  if (z2 <= 0) {
    return -1;
  }

  // Check reprojection error
  cv::Point2f uv1 = this->project(x3D);

  float errX1 = uv1.x - kp1.pt.x;
  float errY1 = uv1.y - kp1.pt.y;

  if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel) {  // Reprojection error is high
    return -1;
  }

  cv::Mat x3D2 = R21 * x3D + t21;
  cv::Point2f uv2 = pCamera2->project(x3D2);

  float errX2 = uv2.x - kp2.pt.x;
  float errY2 = uv2.y - kp2.pt.y;

  if ((errX2 * errX2 + errY2 * errY2) > 5.991 * unc) {  // Reprojection error is high
    return -1;
  }

  p3D = x3D.clone();

  if (pz2) *pz2 = z2;
  return z1;
}
}  // namespace VIEO_SLAM