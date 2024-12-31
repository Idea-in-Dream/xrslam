#ifndef XRSLAM_COMMON_H
#define XRSLAM_COMMON_H

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <forward_list>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <xrslam/xrslam.h>
#include <xrslam/utility/debug.h>

#define XRSLAM_GRAVITY_NOMINAL 9.80665

namespace xrslam {

// 模板别名
// map 和 const_map 是 Eigen::Map 类型的模板别名，分别表示一个非常量和常量的 Eigen 矩阵映射。
// Eigen::Map 是 Eigen 库提供的一种机制，用于在现有内存上创建矩阵或向量的视图，而不需要额外分配新的内存。
// map<T>: 映射到非常量的内存，可以对数据进行修改。
// const_map<T>: 映射到常量的内存，只能读取数据，不能修改。
template <typename T> using map = Eigen::Map<T>;

template <typename T> using const_map = Eigen::Map<const T>;

// 返回一个 size_t 类型的最大值
// 这个值通常被用作无效索引或特殊标记，类似于 NULL 或 nullptr 的作用
inline constexpr size_t nil() { return size_t(-1); }

// compare 是一个模板结构体，用于定义比较两个对象的方法。它的具体实现可能因类型 T 的不同而不同。
template <typename T>
struct compare; /*
    constexpr bool operator()(const T &a, const T &b) const;
*/

// 用于在 STL 容器（如 std::set 或 std::map）中，对指针类型的数据按指针指向的值进行排序。
template <typename T> struct compare<T *> {
    // 它重载了 operator()，接受两个指针 a 和 b，并通过 std::less 比较它们所指向的对象的值。
    constexpr bool operator()(const T *a, const T *b) const {
        // std::less<T>()(*a, *b)：比较指针 a 和 b 所指向的对象。
        return std::less<T>()(*a, *b);
    }
};

struct ImuData {
    double t;
    vector<3> w;
    vector<3> a;
};

template <class FlagEnum> struct Flagged {
    static const size_t flag_num = static_cast<size_t>(FlagEnum::FLAG_NUM);

    Flagged() { flags.reset(); }

    bool operator==(const Flagged &rhs) const { return (flags == rhs.flags); }

    bool flag(FlagEnum f) const { return flags[static_cast<size_t>(f)]; }

    typename std::bitset<flag_num>::reference flag(FlagEnum f) {
        return flags[static_cast<size_t>(f)];
    }

    bool any_of(std::initializer_list<FlagEnum> flags) const {
        return std::any_of(flags.begin(), flags.end(),
                           [this](FlagEnum f) { return flag(f); });
    }

    bool all_of(std::initializer_list<FlagEnum> flags) const {
        return std::all_of(flags.begin(), flags.end(),
                           [this](FlagEnum f) { return flag(f); });
    }

    bool none_of(std::initializer_list<FlagEnum> flags) const {
        return std::none_of(flags.begin(), flags.end(),
                            [this](FlagEnum f) { return flag(f); });
    }

  private:
    std::bitset<flag_num> flags;
};

} // namespace xrslam

#define synchronized(obj_ptr)                                                  \
    if constexpr (auto local_synchronized_lock__ = (obj_ptr)->lock(); true)

#endif // XRSLAM_COMMON_H
