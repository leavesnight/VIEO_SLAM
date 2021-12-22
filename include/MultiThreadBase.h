//
// Created by leavesnight on 2021/12/20.
//

#ifndef VIEO_SLAM_MULTITHREADBASE_H
#define VIEO_SLAM_MULTITHREADBASE_H

#include <mutex>

namespace VIEO_SLAM {
class MutexUsed {
 public:
  template <typename _Mutex>
  using unique_lock = std::unique_lock<_Mutex>;
  using mutex = std::mutex;
};
}

#endif  // VIEO_SLAM_MULTITHREADBASE_H
