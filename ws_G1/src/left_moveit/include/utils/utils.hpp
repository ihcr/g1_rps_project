#ifndef UNITREE_UTILS_HPP
#define UNITREE_UTILS_HPP

namespace unitree {
namespace common {

template<typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

} // namespace common
} // namespace unitree

#endif // UNITREE_UTILS_HPP
