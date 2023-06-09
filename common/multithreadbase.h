//
// Created by leavesnight on 2021/12/20.
//

#pragma once

#include <mutex>

namespace VIEO_SLAM {
class MutexUsed {
 public:
  template <typename _Mutex>
  using unique_lock = std::unique_lock<_Mutex>;
  using mutex = std::mutex;
};

}  // namespace VIEO_SLAM
