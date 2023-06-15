//
// Created by leavesnight on 5/30/23.
//

#pragma once

// notice Get##Name() calls copy constructor when return
#define CREATOR_VAR_MUTEX(Name, Type, Suffix, InitVal) \
  Type Suffix##Name##_ = InitVal;                      \
  mutable mutex mutex##Name##_;
#define CREATOR_GET(Name, Type, Suffix)      \
  Type const& Get##Name(void) const {        \
    unique_lock<mutex> lock(mutex##Name##_); \
    return Suffix##Name##_;                  \
  }
#define CREATOR_SET(Name, Type, Suffix)      \
  void Set##Name(Type const& value) {        \
    unique_lock<mutex> lock(mutex##Name##_); \
    Suffix##Name##_ = value;                 \
  }
#define CREATOR_VAR_MULTITHREADS(Name, Type, Suffix, access_permission, InitVal) \
  access_permission:                                                             \
  CREATOR_VAR_MUTEX(Name, Type, Suffix, InitVal)                                 \
 public:                                                                         \
  CREATOR_GET(Name, Type, Suffix)                                                \
  CREATOR_SET(Name, Type, Suffix)                                                \
  access_permission:
