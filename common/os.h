#pragma once

#include "common/mlog/log.h"

#ifdef WINDOWS
#define NOMINMAX  // to avoid std::max/min ( error in opencv
#include <windows.h>
#include <io.h>
#include <direct.h>
using err_t = DWORD;
using policy_t = DWORD;

using __useconds_t = unsigned int;
inline int usleep(__useconds_t dt_us) {
  DWORD dt_ms = DWORD(dt_us / 1000. + 0.5);
  Sleep(dt_ms);
  return 0;
}
using pid_t = DWORD;
// now use handle as pthread_t
using pthread_t = HANDLE;
inline pid_t gettid() { return GetCurrentThreadId(); }
inline pthread_t pthread_self() { return GetCurrentThread(); }
inline int access(const char* __name, int __type) {
  switch (__type) {
    case 0:  // F_OK
    case 2:  // W_OK
    case 4:  // R_OK
      break;
    default:
      PRINT_ERR_MUTEX("Wrong Usage of universal access mode=" << __type << std::endl);
      exit(-1);
  }
  return _access(__name, __type);
}
inline int mkdir(const char* __path, unsigned int __mode) { return _mkdir(__path); }
#else
#include <unistd.h>
using err_t = int;
using policy_t = int;
#endif
