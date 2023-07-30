//
// Created by leavesnight on 5/30/23.
//

#pragma once

#ifdef WINDOWS
#if defined(COMPILE_LIB_g2o)
#define g2o_API __declspec(dllexport)
#else
#define g2o_API __declspec(dllimport)
#endif
#else
#define g2o_API __attribute__((visibility("default")))
#endif
