//
// Created by leavesnight on 6/6/23.
//

#include <opencv2/core/core.hpp>
#include "camera_pinhole.h"

#include "include/Converter.h"

namespace VIEO_SLAM {
namespace camm {
PinholeCamera::PinholeCamera(cv::FileStorage &fSettings, int id, bool &bmiss_param)
    : Base(id, 0, 0, vector<Tdata>(), SE3data()) {
  string cam_name = "Camera" + (!id ? "" : std::to_string(id + 1));

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
  float fy = node_tmp;
  node_tmp = fSettings[cam_name + ".cx"];
  if (node_tmp.empty()) {
    bmiss_param = true;
    return;
  }
  float cx = node_tmp;
  node_tmp = fSettings[cam_name + ".cy"];
  if (node_tmp.empty()) {
    bmiss_param = true;
    return;
  }
  float cy = node_tmp;

  parameters_ = {fx, fy, cx, cy};

  using std::endl;
  PRINT_INFO_MUTEX(endl << "Camera (Pinhole) Parameters: " << endl);
  PRINT_INFO_MUTEX("- fx: " << fx << endl);
  PRINT_INFO_MUTEX("- fy: " << fy << endl);
  PRINT_INFO_MUTEX("- cx: " << cx << endl);
  PRINT_INFO_MUTEX("- cy: " << cy << endl);

  node_tmp = fSettings[cam_name + ".Trc"];
  if (!node_tmp.empty()) {
    cv::Mat cvTrc = node_tmp.mat();
    if (cvTrc.rows != 3 || cvTrc.cols != 4) {
      std::cerr << "*Trc matrix have to be a 3x4 transformation matrix*" << std::endl;
      bmiss_param = true;
      return;
    }
    SetTrc(SE3data(Sophus::SO3ex<TdataGeo>(Converter::toMatrix3d(cvTrc.rowRange(0, 3).colRange(0, 3)).cast<TdataGeo>()),
                   Converter::toVector3d(cvTrc.col(3)).cast<TdataGeo>()));
  } else {
    PRINT_INFO_MUTEX("Warning:*Trc matrix doesn't exist*" << std::endl);
  }
  PRINT_INFO_MUTEX("- Trc: \n" << Trc_.matrix3x4() << std::endl);

  bmiss_param = false;

  camera_model_ = CameraModel::kPinhole;
}
bool PinholeCamera::ParseCamParamFile(cv::FileStorage &fSettings, int id, Camera::Ptr &pCamInst) {
  bool b_miss_params = false;
  pCamInst = std::make_shared<PinholeCamera>(fSettings, id, b_miss_params);
  using std::cerr;
  using std::endl;
  if (b_miss_params) {
    cerr << "Error: miss params!" << endl;
    return false;
  }

  return true;
}

}  // namespace camm
}  // namespace VIEO_SLAM