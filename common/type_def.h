//
// Created by leavesnight on 6/9/23.
//

#pragma once

namespace VIEO_SLAM {
namespace common {
#define INVALID_TIMESTAMP (-1)
using TimeStamp = double;                      // uint64_t;
constexpr double CoeffTimeStampToSecond = 1.;  // 1.e-9;
constexpr double TS2S(const TimeStamp ts) { return static_cast<double>(ts * CoeffTimeStampToSecond); }
constexpr TimeStamp S2TS(const double ts) { return static_cast<TimeStamp>(ts / CoeffTimeStampToSecond); }

}  // namespace common
}  // namespace VIEO_SLAM
