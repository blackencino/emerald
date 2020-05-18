#pragma once

#include <OpenEXR/ImathVec.h>

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

namespace emerald::util {

//------------------------------------------------------------------------------
// Scalar safe divide
template <typename T>
std::enable_if_t<std::is_integral_v<T>, bool> is_safe_divide(
    [[maybe_unused]] T const, T const denom) {
    return denom != 0;
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, bool> is_safe_divide(
    T const numer, T const denom) {
    if (denom == 0) {
        return false;
    } else if (std::abs(denom) >= 1) {
        return true;
    } else {
        auto const mr = std::abs(denom) / std::numeric_limits<T>::min();
        return (mr > std::abs(numer));
    }
}

template <typename T>
std::enable_if_t<std::is_arithmetic_v<T>, std::optional<T>> safe_divide(
    T const numer, T const denom) {
    return is_safe_divide(numer, denom) ? std::optional<T>{numer / denom}
                                        : std::nullopt;
}

//------------------------------------------------------------------------------
// Vec2 safe divide
template <typename T>
std::enable_if_t<std::is_integral_v<T>, bool> is_safe_divide(
    [[maybe_unused]] Imath::Vec2<T> const, T const denom) {
    return denom != 0;
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, bool> is_safe_divide(
    Imath::Vec2<T> const numer, T const denom) {
    if (denom == 0) {
        return false;
    } else if (std::abs(denom) >= 1) {
        return true;
    } else {
        auto const mr = std::abs(denom) / std::numeric_limits<T>::min();
        return (mr > std::abs(numer[0])) && (mr > std::abs(numer[1]));
    }
}

template <typename T>
std::enable_if_t<std::is_arithmetic_v<T>, std::optional<Imath::Vec2<T>>>
safe_divide(

    Imath::Vec2<T> const numer, T const denom) {
    return is_safe_divide(numer, denom)
               ? std::optional<Imath::Vec2<T>>{numer / denom}
               : std::nullopt;
}

//------------------------------------------------------------------------------
// Vec3 safe divide
template <typename T>
std::enable_if_t<std::is_integral_v<T>, bool> is_safe_divide(
    [[maybe_unused]] Imath::Vec3<T> const, T const denom) {
    return denom != 0;
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, bool> is_safe_divide(
    Imath::Vec3<T> const numer, T const denom) {
    if (denom == 0) {
        return false;
    } else if (std::abs(denom) >= 1) {
        return true;
    } else {
        auto const mr = std::abs(denom) / std::numeric_limits<T>::min();
        return (mr > std::abs(numer[0])) && (mr > std::abs(numer[1])) &&
               (mr > std::abs(numer[2]));
    }
}

template <typename T>
std::enable_if_t<std::is_arithmetic_v<T>, std::optional<Imath::Vec3<T>>>
safe_divide(Imath::Vec3<T> const numer, T const denom) {
    return is_safe_divide(numer, denom)
               ? std::optional<Imath::Vec3<T>>{numer / denom}
               : std::nullopt;
}

}  // namespace emerald::util