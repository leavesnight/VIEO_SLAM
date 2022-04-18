//
// Created by leavesnight on 2021/12/22.
//

#ifndef VIEO_SLAM_COMMON_H
#define VIEO_SLAM_COMMON_H

namespace VIEO_SLAM {
//#define USE_STRATEGY_ABANDON
#ifndef USE_STRATEGY_ABANDON
#define USE_STRATEGY_MIN_DIST
//#define USE_STRATEGY_FIRST_ONE
#endif

//#define CHECK_REPLACE_ALL

// Draw related params
//#define DRAW_ALL_KPS
//#define DRAW_KP2MP_LINE

const double kRatioIMUSigma = 1e6;  // 1e3 / 9;
const double kCoeffDeltatPrior[2] = {kRatioIMUSigma * 1e-3, kRatioIMUSigma * 1e-4};
}  // namespace VIEO_SLAM

#endif  // VIEO_SLAM_COMMON_H
