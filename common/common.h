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
}

#endif  // VIEO_SLAM_COMMON_H
