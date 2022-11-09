//
// Created by leavesnight on 2021/12/22.
//

#ifndef VIEO_SLAM_COMMON_H
#define VIEO_SLAM_COMMON_H

namespace VIEO_SLAM {
#define MUTE_VIEWER
#define TIMER_FLOW

//#define USE_STRATEGY_ABANDON
#ifndef USE_STRATEGY_ABANDON
#define USE_STRATEGY_MIN_DIST
#define USE_STRATEGY_FIRST_ONE
#endif

//#define CHECK_REPLACE_ALL
//#define USE_SIMPLE_REPLACE

// Draw related params
//#define DRAW_ALL_KPS
//#define DRAW_KP2MP_LINE

const double kRatioIMUSigma = 1e6;  // 1e3 / 9;
const double kCoeffDeltatPrior[2] = {kRatioIMUSigma * 1e-3, kRatioIMUSigma * 1e-4};
}  // namespace VIEO_SLAM

// notice Get##Name() calls copy constructor when return
#define CREATOR_VAR_MUTEX(Name, Type, Suffix) \
  Type m##Suffix##Name;                       \
  std::mutex mMutex##Name;
#define CREATOR_GET(Name, Type, Suffix)    \
  Type Get##Name(void) {                   \
    unique_lock<mutex> lock(mMutex##Name); \
    return m##Suffix##Name;                \
  }
#define CREATOR_SET(Name, Type, Suffix)    \
  void Set##Name(Type value) {             \
    unique_lock<mutex> lock(mMutex##Name); \
    m##Suffix##Name = value;               \
  }
#define CREATOR_VAR_MULTITHREADS(Name, Type, Suffix) \
 private:                                            \
  CREATOR_VAR_MUTEX(Name, Type, Suffix)              \
 public:                                             \
  CREATOR_GET(Name, Type, Suffix)                    \
  CREATOR_SET(Name, Type, Suffix)                    \
 private:
#define CREATOR_VAR_MUTEX_INIT(Name, Type, Suffix, InitVal) \
  Type m##Suffix##Name = InitVal;                           \
  std::mutex mMutex##Name;
#define CREATOR_VAR_MULTITHREADS_INIT(Name, Type, Suffix, access_permission, InitVal) \
  access_permission:                                                                  \
  CREATOR_VAR_MUTEX_INIT(Name, Type, Suffix, InitVal)                                 \
 public:                                                                              \
  CREATOR_GET(Name, Type, Suffix)                                                     \
  CREATOR_SET(Name, Type, Suffix)                                                     \
  access_permission:

#endif  // VIEO_SLAM_COMMON_H
