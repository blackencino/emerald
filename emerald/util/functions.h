#pragma once

#include <emerald/util/foundation.h>

#include <algorithm>
#include <type_traits>

namespace emerald {
namespace util {

//-*****************************************************************************
//-*****************************************************************************
// SHOULDER FUNCTION - this implements a curve that's basically just 'x'
// for x <= 0.5, and then asymptotically approaches 1 for any value greater
// than 0.5. This can be used to implement a soft clamp.
template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, T> shoulder(T const x) {
    constexpr T Half = 0.5;
    constexpr T Two = 2.0;

    if (x <= Half) {
        return x;
    } else {
        return Half + (Half * std::tanh(Two * (x - Half)));
    }
}

//-*****************************************************************************
// Limit value to never get larger than maxVal, using shoulder curve.
template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, T> shoulderLimit(T const x,
                                                               T const maxVal) {
    return maxVal * shoulder(x / maxVal);
}

//-*****************************************************************************
// A collection of simple functions borrowed from various places
// to make this library self-sufficient.
//-*****************************************************************************

//-*****************************************************************************
// You'd think instead of doing the a*(1-t) + b*t, it'd be faster
// and one less multiply to do a + (b-a)*t, right? Bad! Increases floating
// point exception occurances. Same as LERP
template <typename T, typename T2>
T mix(T const& a, T const& b, T2 const interp) {
    constexpr T2 one = 1;
    return (a * (one - interp)) + (b * interp);
}

//******************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> radians(
    T const deg) {
    constexpr T TPI = ((T)M_PI);
    constexpr T ONEEIGHTY = 180;
    return TPI * (deg / ONEEIGHTY);
}

//******************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> degrees(
    T const rad) {
    constexpr T TPI = ((T)M_PI);
    constexpr T ONEEIGHTY = 180;
    return ONEEIGHTY * (rad / TPI);
}

//-*****************************************************************************
// f(x) = c0 + c1*x + c2*x^2 + c3*x^3
template <typename T1, typename T2>
T1 cubic(T1 const& coeff0,
         T1 const& coeff1,
         T1 const& coeff2,
         T1 const& coeff3,
         T2 const& t) {
    return coeff0 + t * (coeff1 + t * (coeff2 + t * coeff3));
}

//-*****************************************************************************
// Hermite function.
// f(x) = (c0) + (c1)*x + (c2)*x^2 + (c3)*x^3
// f'(x) = (c1) + 2(c2)x + 3(c3)x^2
// f(0) = A        ->        A = (c0)
// f(1) = B        ->        B = (c0) + (c1) + (c2) + (c3)
// f'(0) = sA      ->       sA = (c1)
// f'(1) = sB      ->       sB = (c1) + 2(c2) + 3(c3)
// B - A - sA = c2 + c3 = "M"
// sB - sA = 2(c2) + 3(c3) = "N"
// 2c2 + 2c3 = 2M
// 2c2 + 3c3 = N
// c3 = N - 2M
// c2 = M - c3
template <typename T1, typename T2>
T1 hermite(T1 const& pointA,
           T1 const& pointB,
           T1 const& slopeA,
           T1 const& slopeB,
           T2 const& t) {
    auto const M = pointB - pointA - slopeA;
    auto const M2 = M * ((T2)2);
    auto const N = slopeB - slopeA;
    auto const c3 = N - M2;
    return cubic(pointA, slopeA, M - c3, c3, t);
}

//******************************************************************************
// Smoothstep function
// Goes from 0 to 1.
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> smoothstep(
    T const t) {
    constexpr T T0 = 0;
    constexpr T T1 = 1;
    constexpr T T2 = 2;
    constexpr T T3 = 3;
    if (t <= T0) {
        return T0;
    } else if (t >= T1) {
        return T1;
    } else {
        return t * t * (T3 - (t * T2));
    }
}

//******************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> smoothstep(
    T const edge0, T const edge1, T const t) {
    return smoothstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> linstep(T const t) {
    constexpr T t0 = 0;
    constexpr T t1 = 1;

    return std::clamp(t, t0, t1);
}

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> linstep(
    T const edge0, T const edge1, T const t) {
    return linstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
// This wraps the input x into the range [lowerBound,upperBound], with periodic
// repeat. So, for example, for ints, a section of the number line, for the
// lowerbound of 2 and an upper bound of 5:
//
// INPUT X: -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10
//  OUTPUT:  5  2  3 4 5 2 3 4 5 2 3 4 5  2
// template <typename T>
// inline T wrap( T i_x, T i_lowerBound, T i_upperBound );

// This version wraps x into the periodic range [0,n). It is the same
// as calling wrap( x, 0, n-1 ), for int-likes,
// and wrap( x, 0, n ) for float-likes
// template <typename T>
// inline T wrap( T x, T n );

//-*****************************************************************************
// This assumes an integer type for T.
template <typename T>
constexpr std::enable_if_t<std::is_integral_v<T> && std::is_signed_v<T>, T>
wrap(T const x, T const lowerBound, T const upperBound) {
    auto const rangeSize = upperBound - lowerBound + 1;

    if (x < lowerBound) {
        auto const x2 = x + rangeSize * (((lowerBound - x) / rangeSize) + 1);
        return lowerBound + (x2 - lowerBound) % rangeSize;
    } else {
        return lowerBound + (x - lowerBound) % rangeSize;
    }
}

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_integral_v<T> && std::is_signed_v<T>, T>
wrap(T const x, T const N) {
    if (x < 0) {
        auto const x2 = x + N * (((-x) / N) + 1);
        return x2 % N;
    } else {
        return x % N;
    }
}  // namespace util

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> wrap(T const x,
                                                                T const n) {
    return x - (n * std::floor(x / n));
}

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> wrap(T const x,
                                                                T const lb,
                                                                T const ub) {
    return lb + wrap(x - lb, ub - lb);
}

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> sqr(T const a) {
    return a * a;
}

//-*****************************************************************************
template <typename T>
constexpr std::enable_if_t<std::is_floating_point_v<T>, T> cube(T const a) {
    return a * a * a;
}

//-*****************************************************************************
//-*****************************************************************************
// BOX INTERSECTION STUFF
//-*****************************************************************************
//-*****************************************************************************

template <typename T>
Imath::Box<Imath::Vec3<T>> BoxIntersection(
    Imath::Box<Imath::Vec3<T>> const& i_a,
    Imath::Box<Imath::Vec3<T>> const& i_b) {
    return Imath::Box<Imath::Vec3<T>>{
        Imath::Vec3<T>{std::max(i_a.min.x, i_b.min.x),
                       std::max(i_a.min.y, i_b.min.y),
                       std::max(i_a.min.z, i_b.min.z)},

        Imath::Vec3<T>{std::min(i_a.max.x, i_b.max.x),
                       std::min(i_a.max.y, i_b.max.y),
                       std::min(i_a.max.z, i_b.max.z)}};
}

template <typename T>
Imath::Box<Imath::Vec3<T>> BoxIntersection(
    Imath::Box<Imath::Vec3<T>> const& i_a,
    Imath::Box<Imath::Vec3<T>> const& i_b,
    Imath::Box<Imath::Vec3<T>> const& i_c) {
    return BoxIntersection<T>(BoxIntersection<T>(i_a, i_b), i_c);
}

}  // namespace util
}  // End namespace emerald
