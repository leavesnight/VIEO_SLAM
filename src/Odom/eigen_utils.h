//
// Created by leavesnight on 2021/4/20.
//

#ifndef VIEO_SLAM_EIGEN_UTILS_H
#define VIEO_SLAM_EIGEN_UTILS_H

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>
#include <list>

#include <Eigen/Dense>

namespace Eigen {

    template<class T>
    using aligned_list = std::list<T, Eigen::aligned_allocator<T> >;

    template <typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

    template <typename T>
    using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

    template <typename K, typename V>
    using aligned_map = std::map<K, V, std::less<K>,
            Eigen::aligned_allocator<std::pair<K const, V>>>;

    template <typename K, typename V>
    using aligned_unordered_map =
    std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
            Eigen::aligned_allocator<std::pair<K const, V>>>;

}  // namespace Eigen

#endif //VIEO_SLAM_EIGEN_UTILS_H
