//
// Created by leavesnight on 5/30/23.
//

#pragma once

#ifdef WINDOWS
#if defined(COMPILE_LIB_DBoW2)
#define DBoW2_API __declspec(dllexport)
#else
#define DBoW2_API __declspec(dllimport)
#endif
#else
#define DBoW2_API __attribute__((visibility("default")))
#endif
