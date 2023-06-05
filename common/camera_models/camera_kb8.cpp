//
// Created by leavesnight on 6/6/23.
//

#include <opencv2/core/core.hpp>
#include "camera_kb8.h"

#include "include/Converter.h"

namespace VIEO_SLAM {
namespace camm {
KB8Camera::KB8Camera(const std::vector<Tdata> &distcoef, cv::FileStorage &fSettings, int id, bool &bmiss_param)
    : Base(fSettings, id, bmiss_param) {
  assert(distcoef.size() == 4);
  parameters_.resize(8);
  for (int i = 0; i < 4; ++i) parameters_[4 + i] = distcoef[i];
  camera_model_ = CameraModel::kKB8;
}
bool KB8Camera::ParseCamParamFile(cv::FileStorage &fSettings, int id, Camera::Ptr &pCamInst) {
  string cam_name = "Camera" + (!id ? "" : std::to_string(id + 1));
  cv::FileNode node_tmp = fSettings[cam_name + ".k1"];
  if (node_tmp.empty()) return false;

  bool b_miss_params = false;

  vector<Tdata> distcoef(4);
  for (int i = 0; i < 4; ++i) {
    distcoef[i] = fSettings[cam_name + ".k" + std::to_string(i + 1)];
  }

  pCamInst = std::make_shared<KB8Camera>(distcoef, fSettings, id, b_miss_params);
  if (b_miss_params) return false;

  using std::endl;
  PRINT_INFO_MUTEX(endl << cam_name << " (KB8) Parameters: " << endl);
  for (int i = 0; i < 4; ++i) {
    PRINT_INFO_MUTEX("- k" + std::to_string(i + 1) + ": " << distcoef[i] << endl);
  }

  int LappingBegin = -1;
  int LappingEnd = -1;

  cv::FileNode node = fSettings[cam_name + ".lappingBegin"];
  if (!node.empty() && node.isInt())
    LappingBegin = node.operator int();
  else
    PRINT_INFO_MUTEX("WARNING: Camera.lappingBegin not correctly defined" << std::endl);
  node = fSettings[cam_name + ".lappingEnd"];
  if (!node.empty() && node.isInt())
    LappingEnd = node.operator int();
  else
    PRINT_INFO_MUTEX("WARNING: Camera.lappingEnd not correctly defined" << std::endl);

  if (!b_miss_params) {
    std::static_pointer_cast<KB8Camera>(pCamInst)->SetvLappingArea(vector<int>({LappingBegin, LappingEnd}));
    PRINT_INFO_MUTEX("- " << cam_name << " Lapping: " << LappingBegin << ", " << LappingEnd << std::endl);
  }

  // TODO: check the input
  return true;
}

}  // namespace camm
}  // namespace VIEO_SLAM