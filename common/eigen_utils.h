//
// Created by leavesnight on 2021/4/20.
//

#pragma once

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>
#include <list>
#include <Eigen/Dense>

namespace Eigen {

template <class _Tp>
using aligned_list = std::list<_Tp, Eigen::aligned_allocator<_Tp>>;

template <typename _Tp>
using aligned_vector = std::vector<_Tp, Eigen::aligned_allocator<_Tp>>;

template <typename _Tp>
using aligned_deque = std::deque<_Tp, Eigen::aligned_allocator<_Tp>>;

template <typename _Key, typename _Tp, typename _Compare = std::less<_Key>>
using aligned_map = std::map<_Key, _Tp, _Compare, Eigen::aligned_allocator<std::pair<_Key const, _Tp>>>;

template <typename _Key, typename _Tp, typename _Hash = std::hash<_Key>, typename _Pred = std::equal_to<_Key>>
using aligned_unordered_map =
    std::unordered_map<_Key, _Tp, _Hash, _Pred, Eigen::aligned_allocator<std::pair<_Key const, _Tp>>>;

}  // namespace Eigen
