//
// Created by leavesnight on 2021/12/21.
//

#ifndef VIEO_SLAM_LOG_H
#define VIEO_SLAM_LOG_H

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace VIEO_SLAM {
enum TYPE_PRINT_LEVEL { PRINT_LEVEL_SILENT = 0, PRINT_LEVEL_INFO = 1, PRINT_LEVEL_DEBUG = 2 };
constexpr int PRINT_LEVEL = PRINT_LEVEL_INFO;  // PRINT_LEVEL_DEBUG;//

extern std::mutex gmutexOUTPUT;
const std::string imu_tightly_debug_path = "/home/leavesnight/tmp/VIEOSLAM/";  //"/backup/imu_tightly/";
}  // namespace VIEO_SLAM

#define PRINT_INFO_BASE(msg, level, foldername, filename)              \
  do {                                                                 \
    if (VIEO_SLAM::PRINT_LEVEL >= level) {                             \
      if ("" != foldername) {                                          \
        std::string debug_file = std::string(foldername) + filename;   \
        std::ofstream fout(debug_file, std::ios::out | std::ios::app); \
        fout << msg;                                                   \
      } else                                                           \
        std::cout << msg;                                              \
    }                                                                  \
  } while (0)
#define PRINT_INFO_MUTEX_BASE(msg, level, foldername, filename)        \
  do {                                                                 \
    if (VIEO_SLAM::PRINT_LEVEL >= level) {                             \
      if ("" != foldername) {                                          \
        std::unique_lock<std::mutex> lock(VIEO_SLAM::gmutexOUTPUT);    \
        std::string debug_file = std::string(foldername) + filename;   \
        std::ofstream fout(debug_file, std::ios::out | std::ios::app); \
        fout << msg;                                                   \
      } else                                                           \
        std::cout << msg;                                              \
    }                                                                  \
  } while (0)
#define PRINT_INFO(msg) PRINT_INFO_BASE(msg, VIEO_SLAM::PRINT_LEVEL_INFO, "", "")
#define PRINT_INFO_MUTEX(msg) PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::PRINT_LEVEL_INFO, "", "")
#define PRINT_INFO_FILE(msg, foldername, filename) \
  PRINT_INFO_BASE(msg, VIEO_SLAM::PRINT_LEVEL_INFO, foldername, filename)
#define PRINT_INFO_FILE_MUTEX(msg, foldername, filename) \
  PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::PRINT_LEVEL_INFO, foldername, filename)
#define PRINT_DEBUG_INFO(msg, foldername, filename) \
  PRINT_INFO_BASE(msg, VIEO_SLAM::PRINT_LEVEL_DEBUG, foldername, filename)
#define PRINT_DEBUG_INFO_MUTEX(msg, foldername, filename) \
  PRINT_INFO_MUTEX_BASE(msg, VIEO_SLAM::PRINT_LEVEL_DEBUG, foldername, filename)
#define CLEAR_DEBUG_INFO(msg, foldername, filename)                \
  do {                                                             \
    if ("" != foldername) {                                        \
      std::string debug_file = std::string(foldername) + filename; \
      std::ofstream fout(debug_file, std::ios::out);               \
      fout << msg;                                                 \
    }                                                              \
  } while (0)

class Timer {
  using clock = std::chrono::steady_clock;
  clock::time_point t0_, t1_;
  static std::vector<double> sum_dts_;
  static std::vector<size_t> num_dts_;

  std::vector<double> calc_time_output(double dt, size_t i, const std::string& filename = "",
                                       const std::string& prefix = "", bool bsinglethread = true) {
    std::vector<double> tm_costs = {dt, sum_dts_[i] / num_dts_[i]};
    if ("" != filename) {
      if (bsinglethread)
        PRINT_INFO_FILE(prefix << tm_costs[0] << ",avg=" << tm_costs[1] << std::endl, VIEO_SLAM::imu_tightly_debug_path,
                        filename);
      else
        PRINT_INFO_FILE_MUTEX(prefix << tm_costs[0] << ",avg=" << tm_costs[1] << std::endl,
                              VIEO_SLAM::imu_tightly_debug_path, filename);
    }
    return tm_costs;
  }
  std::vector<double> GetDT(int mode, size_t i, const std::string& filename = "", const std::string& prefix = "",
                            bool bsinglethread = true) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    auto t2 = clock::now();
    double dt;
    if (0 == mode) {
      t1_ = t2;
      dt = duration_cast<duration<double>>(t1_ - t0_).count();
    } else {
      dt = duration_cast<duration<double>>(t2 - t1_).count();
      t1_ = t2;
    }
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

#endif  // VIEO_SLAM_LOG_H
