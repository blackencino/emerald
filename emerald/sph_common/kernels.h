#pragma once

#include <emerald/sph_common/types.h>
#include <emerald/util/functions.h>
#include <emerald/util/safe_divide.h>

namespace emerald::sph_common {
namespace kernels {

using namespace emerald::util;

static constexpr float _pi = 3.14159274101257324218750f;

// These are the standard SPH composite curves for kernels

inline constexpr float _wa(float const q) {
    return 1.0f + q * q * (q * (3.0f / 4.0f) - (3.0f / 2.0f));
}

inline constexpr float _wb(float const q_in) {
    auto const q = 2.0f - q_in;
    return (q * q * q) / 4.0f;
}

inline constexpr float W(float const r, float const h) {
    float const q = r / h;
    float const k = h * h * h * _pi;
    if (q <= 1) {
        return _wa(q) / k;
    } else if (q <= 2) {
        return _wb(q) / k;
    } else {
        return 0.0f;
    }
}

inline constexpr float _dwa(float const r, float const h) {
    return -3.0f * r / (h * h) + (9.0f / 4.0f) * r * r / (h * h * h);
}

inline constexpr float _dwb(float const r, float const h) {
    return -3.0f * sqr(2.0f - (r / h)) / (4.0f * h);
}

inline constexpr float dW(float const r, float const h) {
    float const q = r / h;
    float const k = h * h * h * _pi;
    if (q <= 1) {
        return _dwa(r, h) / k;
    } else if (q <= 2) {
        return _dwb(r, h) / k;
    } else {
        return 0.0f;
    }
}

inline V2f GradW(const V2f& dP, float const h) {
    float const r = dP.length();
    if (is_safe_divide(dP, r)) {
        return (dP / r) * dW(r, h);
    } else {
        return V2f{0.0f, 0.0f};
    }
}

}  // namespace kernels
}  // namespace emerald::sph_common
