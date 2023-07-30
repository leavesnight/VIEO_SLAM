//
// Created by leavesnight on 5/30/23.
//

#pragma once

#ifdef WINDOWS
#if defined(COMPILE_LIB_COMMON)
#define COMMON_API __declspec(dllexport)
#else
#define COMMON_API __declspec(dllimport)
#endif
#else
#define COMMON_API __attribute__((visibility("default")))
#endif
