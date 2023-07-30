//
// Created by leavesnight on 2021/12/21.
//

#pragma once

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>
#include "common/interface.h"

#ifdef NDEBUG
// assert in Release, should be put at the last include <assert.h> in .h/.cpp
#undef NDEBUG
#include <assert.h>
// if still not assert takes effects, annotate this
//#define NDEBUG
#else
#include <assert.h>
#endif

namespace VIEO_SLAM {
namespace mlog {
enum TYPE_PRINT_LEVEL { PRINT_LEVEL_ERROR = 0, PRINT_LEVEL_INFO = 1, PRINT_LEVEL_DEBUG = 2 };
constexpr int PRINT_LEVEL = PRINT_LEVEL_INFO;  // PRINT_LEVEL_DEBUG;//

typedef enum kVerboseLevel { kVerbRel, kVerbDeb, kVerbFull } ekVerboseLevel;

// to files log related
const std::string vieo_slam_debug_path = "/home/leavesnight/tmp/VIEOSLAM/";
const std::string online_calibrate_debug_path = "/home/leavesnight/tmp/VIEOSLAM/OnlineCalib/";
const std::string hard_case_debug_path = "/home/leavesnight/tmp/VIEOSLAM/HardCase/";

// multi-threads log related
extern COMMON_API std::mutex gmutexOUTPUT;

// macro is invalid for namespace, but put here for Timer definition
#define PRINT_INFO_BASE(msg, level, foldername, filename)              \
  do {                                                                 \
    if constexpr (VIEO_SLAM::mlog::PRINT_LEVEL >= level) {             \
      auto foldername_str = std::string(foldername);                   \
      if (!foldername_str.empty()) {                                   \
        std::string debug_file = foldername_str + filename;            \
        std::ofstream fout(debug_file, std::ios::out | std::ios::app); \
        fout << msg;                                                   \
      } else {                                                         \
        if constexpr (VIEO_SLAM::mlog::PRINT_LEVEL_ERROR == level)     \
          std::cerr << msg;                                            \
        else                                                           \
          std::cout << msg;                                            \
      }                                                                \
    }                                                                  \
  } while (0)
#define PRINT_INFO_MUTEX_BASE(msg, level, foldername, filename)           \
  do {                                                                    \
    if constexpr (VIEO_SLAM::mlog::PRINT_LEVEL >= level) {                \
      auto foldername_str = std::string(foldername);                      \
      if (!foldername_str.empty()) {                                      \
        std::unique_lock<std::mutex> lock(VIEO_SLAM::mlog::gmutexOUTPUT); \
        std::string debug_file = foldername_str + filename;               \
        std::ofstream fout(debug_file, std::ios::out | std::ios::app);    \
        fout << msg;                                                      \
      } else {                                                            \
        if constexpr (VIEO_SLAM::mlog::PRINT_LEVEL_ERROR == level)        \
          std::cerr << msg;                                               \
        else                                                              \
          std::cout << msg;                                               \
      }                                                                   \
    }                                                                     \
  } while (0)
#define PRINT_ERR(msg) PRINT_INFO_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_ERROR, "", "")
#define PRINT_ERR_MUTEX(msg) PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_ERROR, "", "")
#define PRINT_INFO(msg) PRINT_INFO_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_INFO, "", "")
#define PRINT_INFO_MUTEX(msg) PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_INFO, "", "")
#define PRINT_DEBUG(msg) PRINT_INFO_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_DEBUG, "", "")
#define PRINT_DEBUG_MUTEX(msg) PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_DEBUG, "", "")
#define PRINT_INFO_FILE(msg, foldername, filename) \
  PRINT_INFO_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_INFO, foldername, filename)
#define PRINT_INFO_FILE_MUTEX(msg, foldername, filename) \
  PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_INFO, foldername, filename)
#define PRINT_DEBUG_FILE(msg, foldername, filename) \
  PRINT_INFO_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_DEBUG, foldername, filename)
#define PRINT_DEBUG_FILE_MUTEX(msg, foldername, filename) \
  PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::mlog::PRINT_LEVEL_DEBUG, foldername, filename)
#define CLEAR_INFO_FILE(msg, foldername, filename)        \
  do {                                                    \
    auto foldername_str = std::string(foldername);        \
    if (!foldername_str.empty()) {                        \
      std::string debug_file = foldername_str + filename; \
      std::ofstream fout(debug_file, std::ios::out);      \
      fout << msg;                                        \
    }                                                     \
  } while (0)

// colorful cout related, \e not work on git bash of windows
#define redSTR "\033[31m"
#define brightredSTR "\033[31;1m"
#define greenSTR "\033[32m"
#define brightgreenSTR "\033[32;1m"
#define blueSTR "\033[34m"
#define brightblueSTR "\033[34;1m"
#define yellowSTR "\033[33;1m"
#define brownSTR "\033[33m"
#define azureSTR "\033[36;1m"
#define whiteSTR "\033[0m"

class Timer {
  using clock = std::chrono::steady_clock;
  clock::time_point t0_, t1_;
  static std::vector<double> sum_dts_;
  static std::vector<size_t> num_dts_;

  static std::vector<double> calc_time_output(double dt, size_t i, const std::string& filename = "",
                                              const std::string& prefix = "", bool bsinglethread = true) {
    std::vector<double> tm_costs = {dt, sum_dts_[i] / num_dts_[i]};
    if (!filename.empty()) {
      if (bsinglethread)
        PRINT_INFO_FILE(prefix << tm_costs[0] << ",avg=" << tm_costs[1] << std::endl, vieo_slam_debug_path, filename);
      else
        PRINT_INFO_FILE_MUTEX(prefix << tm_costs[0] << ",avg=" << tm_costs[1] << std::endl, vieo_slam_debug_path,
                              filename);
    }
    return tm_costs;
  }
  std::vector<double> GetDT(int mode, size_t i, const std::string& filename = "", const std::string& prefix = "",
                            bool bsinglethread = true) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    auto t2 = clock::now(), t1 = !mode ? t0_ : t1_;
    double dt;
    dt = duration_cast<duration<double>>(t2 - t1).count();
    t1_ = t2;
    if (sum_dts_.size() <= i) {
      sum_dts_.resize(i + 1, 0);
      num_dts_.resize(i + 1, 0);
    }
    sum_dts_[i] += dt;
    ++num_dts_[i];
    return calc_time_output(dt, i, filename, prefix, bsinglethread);
  }

 public:
  Timer() : t0_(clock::now()) {}

  std::vector<double> GetDTfromInit(size_t i = 0, const std::string& filename = "", const std::string& prefix = "",
                                    bool bsinglethread = true) {
    return GetDT(0, i, filename, prefix, bsinglethread);
  }
  std::vector<double> GetDTfromLast(size_t i = 1, const std::string& filename = "", const std::string& prefix = "",
                                    bool bsinglethread = true) {
    return GetDT(1, i, filename, prefix, bsinglethread);
  }
};
}  // namespace mlog
}  // namespace VIEO_SLAM
