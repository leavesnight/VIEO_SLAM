//
// Created by leavesnight on 6/26/23.
//

#pragma once

#include <string>
#include <opencv2/core/core.hpp>

namespace VIEO_SLAM {
class Map;

namespace pcl {
void SaveMapPCL(const std::string &filename, int sensor, Map *pmap, cv::FileStorage &fsettings);
}

}  // namespace VIEO_SLAM
