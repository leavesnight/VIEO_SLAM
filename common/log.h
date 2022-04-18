//
// Created by leavesnight on 2021/12/21.
//

#ifndef VIEO_SLAM_LOG_H
#define VIEO_SLAM_LOG_H

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>

namespace VIEO_SLAM {
enum TYPE_PRINT_LEVEL { PRINT_LEVEL_SILENT = 0, PRINT_LEVEL_INFO = 1, PRINT_LEVEL_DEBUG = 2 };
constexpr int PRINT_LEVEL = PRINT_LEVEL_INFO;  // PRINT_LEVEL_DEBUG;//

extern std::mutex gmutexOUTPUT;
const std::string imu_tightly_debug_path = "/home/leavesnight/tmp/VIEOSLAM/";  //"/backup/imu_tightly/";
}  // namespace VIEO_SLAM

#define PRINT_INFO_BASE(msg, level, foldername, filename)              \
  do {                                                                 \
    if (VIEO_SLAM::PRINT_LEVEL >= level) {              \
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

#endif  // VIEO_SLAM_LOG_H
