//
// Created by leavesnight on 2021/12/20.
//

#pragma once

#include <mutex>
#include <list>
#include <thread>
#include <unistd.h>  // for usleep && sysconf
#include <string>
#include "common/macro_creator.h"
#include "common/interface.h"

namespace VIEO_SLAM {
class MutexUsed {
 public:
  template <typename _Mutex>
  using unique_lock = std::unique_lock<_Mutex>;
  using mutex = std::mutex;
};

namespace multithread {
enum AlgEventType : uint8_t { THREAD_POLICY = 0, DELAY_ODOM, SKIP_IMG, THREAD_CRASH };
typedef struct _AlgEvent {
  AlgEventType event_type_;
  void *data_;
} AlgEvent;
enum AlgTheadType : uint8_t { THREAD_UNKNOWN, THREAD_FE, THREAD_BE, THREAD_ODOM, THREAD_BE_GBA, THREAD_BE_LBA };
typedef struct _ThreadPolicyInfo {
  pid_t tid_;       // thread id
  pthread_t ptid_;  // posix thread id
  AlgTheadType thread_type_ = THREAD_UNKNOWN;
  int policy_;  // thread schedule strategy
  int priority_;
  size_t affinity_mask_;
} ThreadPolicyInfo;

COMMON_API void alg_event_listener(AlgEvent *event);
COMMON_API void SetAffinity(multithread::ThreadPolicyInfo &event_info);
}  // namespace multithread

class KeyFrame;
class MultiThreadBase : public MutexUsed {
 public:
  template <typename _Tp>
  using list = std::list<_Tp>;
  using thread = std::thread;

  virtual void SetThreadPolicy(const std::string &settings_path, const std::string &thread_type);
  virtual ~MultiThreadBase();

  unsigned int sleep_time_ = 3000;
  virtual void RequestReset(int8_t id_cam = -1);

 protected:  // multi threads related params
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
  virtual void SetFinish();

  virtual void Run() = 0;

 public:
  CREATOR_GET(finish, bool, b)
  CREATOR_SET(finish_request, bool, b)

  int8_t verbose_ = 1;  // 0;

 protected:  // thread policy/bind related
  multithread::ThreadPolicyInfo event_info_;
  virtual void SetAffinity() { multithread::SetAffinity(event_info_); }
};

}  // namespace VIEO_SLAM
