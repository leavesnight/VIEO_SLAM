//
// Created by leavesnight on 2021/12/21.
//

#include "common/log.h"

namespace VIEO_SLAM {
std::mutex gmutexOUTPUT;
}

std::vector<double> Timer::sum_dts_;
std::vector<size_t> Timer::num_dts_;
