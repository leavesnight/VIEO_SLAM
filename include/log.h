//
// Created by leavesnight on 2021/12/21.
//

#ifndef VIEO_SLAM_LOG_H
#define VIEO_SLAM_LOG_H

#include <fstream>
#include <mutex>

namespace VIEO_SLAM {
extern std::mutex gmutexOUTPUT;
const std::string imu_tightly_debug_path = "/home/leavesnight/tmp/VIEOSLAM/"; //"/backup/imu_tightly/";
#define PRINT_DEBUG_INFO(msg, foldername, filename)                  \
  do {                                                               \
    if (foldername != "") {                                          \
      std::string debug_file = foldername + filename;                \
      std::ofstream fout(debug_file, std::ios::out | std::ios::app); \
      fout << msg;                                                   \
    } else                                                           \
      std::cout << msg;                                              \
  } while (0);
#define PRINT_DEBUG_INFO_MUTEX(msg, foldername, filename)            \
  do {                                                               \
    if (foldername != "") {                                          \
      std::unique_lock<std::mutex> lock(gmutexOUTPUT);               \
      std::string debug_file = foldername + filename;                \
      std::ofstream fout(debug_file, std::ios::out | std::ios::app); \
      fout << msg;                                                   \
    } else                                                           \
      std::cout << msg;                                              \
  } while (0);
#define PRINT_DEBUG_INFO_MUTEX(msg, foldername, filename) ;
#define CLEAR_DEBUG_INFO(msg, foldername, filename)   \
  do {                                                \
    if (foldername != "") {                           \
      std::string debug_file = foldername + filename; \
      std::ofstream fout(debug_file, std::ios::out);  \
      fout << msg;                                    \
    }                                                 \
  } while (0);
}

#endif  // VIEO_SLAM_LOG_H
