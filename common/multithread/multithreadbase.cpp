//
// Created by leavesnight on 6/25/23.
//

#include <sstream>
#include <opencv2/core/persistence.hpp>
#include "multithreadbase.h"
#include "common/mlog/log.h"

namespace VIEO_SLAM {
namespace multithread {
void alg_event_listener(AlgEvent *event) {
  if (!event) {
    assert(0 && "Wrong usage of alg_event_listener");
    return;
  }
  using std::endl;
  using std::hex;
  using std::max;
  using std::min;
  using std::string;
  using std::stringstream;
  using std::to_string;
  switch (event->event_type_) {
    case THREAD_POLICY: {
      ThreadPolicyInfo *pparams = (ThreadPolicyInfo *)event->data_;
      if (!pparams) {
        assert(0 && "Wrong event->data of alg_event_listener");
      }
      auto tid = pparams->tid_;
      int err_no;
      string prefix_thread = to_string(pparams->thread_type_) + ",tid=" + to_string(tid);
#if defined(SET_AFFINITY_LINUX)
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      size_t num_cores = sysconf(_SC_NPROCESSORS_CONF);
      for (int i = 0; i < min(num_cores, 8 * sizeof(pparams->affinity_mask_)); ++i) {
        if ((1 << i) & pparams->affinity_mask_) CPU_SET(i, &cpuset);
      }
      // syscall could work with size_t(<size_t also ok) cpuset, but we recommend to use sched_setaffinity
      // syscall(__NR_sched_setaffinity, tid, sizeof(cpuset), &cpuset);  //cannot be cpuset
      err_no = sched_setaffinity(tid, sizeof(cpuset), &cpuset);
      assert(!err_no);
      // int num_copied_in_mask = syscall(__NR_sched_getaffinity, tid, sizeof(cpuset), &cpuset); // should be 8
      err_no = sched_getaffinity(tid, sizeof(cpuset), &cpuset);
      assert(!err_no);
      stringstream sstr_out_in, sstr_out;
      sstr_out_in << hex << pparams->affinity_mask_;
      for (int i = max((int)(sizeof(pparams->affinity_mask_) / sizeof(cpuset.__bits[0])) - 1, 0); i >= 0; --i) {
        sstr_out << hex << cpuset.__bits[i];
      }
      PRINT_INFO_FILE_MUTEX(
          prefix_thread << ": set affinity=0x" << sstr_out_in.str() << ",real=0x" << sstr_out.str() << endl,
          mlog::vieo_slam_debug_path, "alg_event.txt");
#endif
      sched_param param;
      param.sched_priority = pparams->priority_;
      // To get root priority
      err_no = sched_setscheduler(tid, pparams->policy_, &param);
      if (err_no) {
        perror(prefix_thread.c_str());
        // PRINT_ERR_MUTEX(errno << endl);
        return;
      }
      err_no = sched_getparam(tid, &param);
      assert(!err_no);
      PRINT_INFO_FILE_MUTEX("set th_name=" << (int)pparams->thread_type_ << ",id(t/pt)=" << pparams->tid_ << "/"
                                           << pparams->ptid_ << ",policy=" << pparams->policy_
                                           << ",real=" << sched_getscheduler(tid) << ",pri=" << pparams->priority_
                                           << ",real=" << param.sched_priority << endl,
                            mlog::vieo_slam_debug_path, "alg_event.txt");
    } break;
    case DELAY_ODOM:
    case SKIP_IMG:
      PRINT_INFO_FILE_MUTEX(string((char *)event->data_) << endl, mlog::vieo_slam_debug_path, "alg_event.txt");
      break;
    case THREAD_CRASH:
      PRINT_INFO_FILE_MUTEX(string((char *)event->data_) << endl, mlog::vieo_slam_debug_path, "alg_event.txt");
      assert(0 && "Crash!");
      break;
    default:
      PRINT_INFO_FILE_MUTEX("Unimplemented func. of alg_event_listener" << endl, mlog::vieo_slam_debug_path,
                            "alg_event.txt");
  }
}
void SetAffinity(multithread::ThreadPolicyInfo &event_info) {
  event_info.tid_ = gettid();
  event_info.ptid_ = pthread_self();
  multithread::AlgEvent event;
  event.event_type_ = multithread::THREAD_POLICY;
  event.data_ = &event_info;
  alg_event_listener(&event);
}

}  // namespace multithread

void MultiThreadBase::SetThreadPolicy(const std::string &settings_path, const std::string &thread_type) {
  cv::FileStorage fsettings(settings_path, cv::FileStorage::READ);
  // bind to assigned core
  auto node_tmp = fsettings[thread_type + ".processor_ids"];
  size_t num_cores = sysconf(_SC_NPROCESSORS_CONF);
  event_info_.affinity_mask_ = node_tmp.empty() ? ((size_t)(0x1 << num_cores) - 1) : (size_t)(int)node_tmp;
  node_tmp = fsettings[thread_type + ".priority"];
  event_info_.priority_ = node_tmp.empty() ? 47 : (size_t)(int)node_tmp;
  if (multithread::THREAD_UNKNOWN == event_info_.thread_type_) event_info_.thread_type_ = multithread::THREAD_BE;
  int priority_max_rr = sched_get_priority_max(SCHED_RR);
  if (event_info_.priority_ > priority_max_rr) {
    PRINT_INFO_FILE_MUTEX("th_name=" << (int)event_info_.thread_type_
                                     << ",SCHED_FIFO, priority_min/max_rr=" << sched_get_priority_min(SCHED_RR) << "/"
                                     << priority_max_rr << ",min/max_fifo=" << sched_get_priority_min(SCHED_FIFO) << "/"
                                     << sched_get_priority_max(SCHED_FIFO) << std::endl,
                          mlog::vieo_slam_debug_path, "alg_event.txt");
    event_info_.policy_ = SCHED_FIFO;
    event_info_.priority_ -= priority_max_rr;
  } else
    event_info_.policy_ = SCHED_RR;
}
MultiThreadBase::~MultiThreadBase() {
  if (pthread_) {
    Setfinish_request(true);
    if (pthread_->joinable()) pthread_->join();
    delete pthread_;
    pthread_ = nullptr;
  }
}

void MultiThreadBase::RequestReset(const int8_t id_cam) {
  if (Getfinish()) return;
  Setreset_id_cam_(id_cam);
  Setreset(true);
  for (;;) {
    // if breset_ changes from true to false, resetting is finished
    if (!Getreset()) break;
    usleep(sleep_time_);
  }
}

void MultiThreadBase::SetFinish() {
  unique_lock<mutex> lock(mutexfinish_);
  bfinish_ = true;
}

}  // namespace VIEO_SLAM
