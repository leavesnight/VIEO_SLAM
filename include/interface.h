//
// Created by leavesnight on 5/30/23.
//

#pragma once

#ifdef WINDOWS
#if defined(COMPILE_LIB_VIEO_SLAM)
#define VIEO_SLAM_API __declspec(dllexport)
#else
#define VIEO_SLAM_API __declspec(dllimport)
#endif
#else
#define VIEO_SLAM_API __attribute__((visibility("default")))
#endif
