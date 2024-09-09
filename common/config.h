//
// Created by leavesnight on 2021/12/22.
//

#pragma once

namespace VIEO_SLAM {
//#define TIMER_FLOW

//#define USE_STRATEGY_ABANDON
#ifndef USE_STRATEGY_ABANDON
#define USE_STRATEGY_MIN_DIST
#endif

//#define CHECK_REPLACE_ALL
//#define USE_SIMPLE_REPLACE

// Draw related params
//#define DRAW_ALL_KPS
//#define DRAW_KP2MP_LINE

// Strategy related params
// BA with IMU related params, dataset uses 1e4, but real fast motion requires ~1e2
constexpr double kRatioIMUSigma = 1e3 / 9;  // 1e4;  //
// g then a
constexpr double kCoeffDeltatPrior[2] = {kRatioIMUSigma, kRatioIMUSigma * 1e-3};  // 1e-4};
}  // namespace VIEO_SLAM
