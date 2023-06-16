//
// Created by leavesnight on 2021/12/20.
//

#pragma once

#include <mutex>
#include <list>
#include <thread>
#include <unistd.h>  // for usleep
#include "macro_creator.h"

namespace VIEO_SLAM {
class MutexUsed {
 public:
  template <typename _Mutex>
  using unique_lock = std::unique_lock<_Mutex>;
  using mutex = std::mutex;
};

class KeyFrame;
class MultiThreadBase : public MutexUsed {
 public:
  template <typename _Tp>
  using list = std::list<_Tp>;
  using thread = std::thread;

  virtual ~MultiThreadBase() {
    if (pthread_) {
      Setfinish_request(true);
      if (pthread_->joinable()) pthread_->join();
      delete pthread_;
      pthread_ = nullptr;
    }
  }

  virtual void RequestReset(const int8_t id_cam = -1) {
    if (Getfinish()) return;
    Setreset_id_cam_(id_cam);
    Setreset(true);
    for (;;) {
      // if breset_ changes from true to false, resetting is finished
      if (!Getreset()) break;
      usleep(3000);
    }
  }

 protected:  // thread related params
  // fixed && unfixed curkfs
  CREATOR_VAR_MULTITHREADS(CurrentKeyFrames, list<KeyFrame *>, p, protected, list<KeyFrame *>())
  CREATOR_VAR_MUTEX(finish, bool, b, true)           // checked/get by System.cc
  CREATOR_VAR_MUTEX(finish_request, bool, b, false)  // requested/set by System.cc
  // for reset LocalMapping variables
  CREATOR_VAR_MULTITHREADS(reset, bool, b, protected, false)
  CREATOR_VAR_MULTITHREADS(reset_id_cam_, int8_t, , protected, int8_t())

  // thread pointer maintained here
  thread *pthread_ = nullptr;

  CREATOR_GET(finish_request, bool, b)
  virtual void SetFinish() {
    unique_lock<mutex> lock(mutexfinish_);
    bfinish_ = true;
  }

  virtual void Run() = 0;

 public:
  CREATOR_GET(finish, bool, b)
  CREATOR_SET(finish_request, bool, b)

  int8_t verbose_ = 1;  // 0;
};

}  // namespace VIEO_SLAM
