//
// Created by leavesnight on 5/30/23.
//

#pragma once

#ifdef _MSC_VER
#define COMMON_API __declspec(dllexport)
#else
#define COMMON_API __attribute__((visibility("default")))
#endif
