//
// Created by leavesnight on 2021/12/21.
//

#include "log.h"

namespace VIEO_SLAM {
namespace mlog {
std::mutex gmutexOUTPUT;

std::vector<double> Timer::sum_dts_;
std::vector<size_t> Timer::num_dts_;
}  // namespace mlog
}  // namespace VIEO_SLAM
