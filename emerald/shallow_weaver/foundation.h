#pragma once

#include <emerald/util/foundation.h>

#define gsl_FEATURE_MAKE_SPAN_TO_STD 20
#include <gsl/gsl-lite.hpp>

#include <array>

namespace emerald::shallow_weaver {

using namespace emerald::util;

template <typename T>
using span = gsl::span<T>;

enum class Time_integration { FIRST_ORDER, RUNGE_KUTTA_2, RUNGE_KUTTA_4 };

using int2 = std::array<int, 2>;
using int4 = std::array<int, 4>;
using float2 = std::array<float, 2>;
using float4 = std::array<float, 4>;

}  // namespace emerald::shallow_weaver