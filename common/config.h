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

const double kRatioIMUSigma = 1e4;                                            // 1e3 / 9;
const double kCoeffDeltatPrior[2] = {kRatioIMUSigma * 1e-3, kRatioIMUSigma};  // 1e-4};
}  // namespace VIEO_SLAM
