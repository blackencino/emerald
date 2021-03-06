#pragma once

#include <emerald/noise/foundation.h>

namespace emerald {
namespace noise {

//-*****************************************************************************
// 2D noise
double simplexNoise(double x, double y);

inline double simplexNoise(const V2d& v) {
    return simplexNoise(v.x, v.y);
}

inline float simplexNoise(const V2f& v) {
    return (float)simplexNoise((double)v.x, (double)v.y);
}

//-*****************************************************************************
// 3D noise
double simplexNoise(double x, double y, double z);

inline double simplexNoise(const V3d& v) {
    return simplexNoise(v.x, v.y, v.z);
}

inline float simplexNoise(const V3f& v) {
    return (float)simplexNoise((double)v.x, (double)v.y, (double)v.z);
}

//-*****************************************************************************
// 4D noise
double simplexNoise(double x, double y, double z, double w);

inline double simplexNoise(const V3d& v, double t) {
    return simplexNoise(v.x, v.y, v.z, t);
}

inline float simplexNoise(const V3f& v, float t) {
    return (float)simplexNoise(
        (double)v.x, (double)v.y, (double)v.z, (double)t);
}

//-*****************************************************************************
// This class can be used as a functor.
template <class T>
class SimplexNoise {
public:
    typedef T value_type;
    typedef T output_type;
    typedef Vec2<T> point_type_2d;
    typedef Vec3<T> point_type_3d;
    typedef SimplexNoise<T> this_type;

    SimplexNoise() {
    }

    // 2D noise
    T operator()(T x, T y) const {
        return (T)simplexNoise((double)x, (double)y);
    }

    T operator()(const Vec2<T>& v) const {
        return (T)simplexNoise((double)v.x, (double)v.y);
    }

    // 3D noise
    T operator()(T x, T y, T z) const {
        return (T)simplexNoise((double)x, (double)y, (double)z);
    }

    T operator()(const Vec3<T>& v) const {
        return (T)simplexNoise((double)v.x, (double)v.y, (double)v.z);
    }

    // 4D noise
    T operator()(T x, T y, T z, T w) const {
        return (T)simplexNoise((double)x, (double)y, (double)z, (double)w);
    }

    T operator()(const Vec3<T>& v, T t) const {
        return (T)simplexNoise(
            (double)v.x, (double)v.y, (double)v.z, (double)t);
    }
};

//-*****************************************************************************
// Handy typedefs
typedef SimplexNoise<float> SimplexNoisef;
typedef SimplexNoise<double> SimplexNoised;

//-*****************************************************************************
template <typename PATTERN>
class Abs {
public:
    typedef PATTERN pattern_type;
    typedef typename PATTERN::value_type value_type;
    typedef typename PATTERN::value_type T_t;
    typedef typename PATTERN::output_type output_type;
    typedef Imath::Vec2<T_t> point_type_2d;
    typedef Imath::Vec3<T_t> point_type_3d;
    typedef Abs<PATTERN> this_type;

    Abs(PATTERN pattern)
      : m_pattern(pattern) {
    }

    // 2D noise
    T_t operator()(T_t x, T_t y) const {
        return Imath::Math<T_t>::std::abs(m_pattern(x, y));
    }

    T_t operator()(const Vec2<T_t>& v) const {
        return Imath::Math<T_t>::std::abs(m_pattern(v));
    }

    // 3D noise
    T_t operator()(T_t x, T_t y, T_t z) const {
        return Imath::Math<T_t>::std::abs(m_pattern(x, y, z));
    }

    T_t operator()(const Vec3<T_t>& v) const {
        return Imath::Math<T_t>::std::abs(m_pattern(v));
    }

    // 4D noise
    T_t operator()(T_t x, T_t y, T_t z, T_t w) const {
        return Imath::Math<T_t>::std::abs(m_pattern(x, y, z, w));
    }

    T_t operator()(const Vec3<T_t>& v, T_t t) const {
        return Imath::Math<T_t>::std::abs(m_pattern(v, t));
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
template <typename PATTERN>
class XformEvolveBiasGain {
public:
    typedef PATTERN pattern_type;
    typedef typename PATTERN::value_type value_type;
    typedef typename PATTERN::value_type T_t;
    typedef typename PATTERN::output_type output_type;
    typedef Imath::Matrix44<T_t> matrix_type;
    typedef Imath::Vec2<T_t> point_type_2d;
    typedef Imath::Vec3<T_t> point_type_3d;
    typedef XformEvolveBiasGain<PATTERN> this_type;

    XformEvolveBiasGain(PATTERN pattern,
                        const matrix_type& i_xform = matrix_type(),
                        value_type i_evolveRate = 1.0,
                        value_type i_bias = 0.0,
                        value_type i_gain = 1.0)
      : m_pattern(pattern)
      , m_xform(i_xform)
      , m_evolveRate(i_evolveRate)
      , m_bias(i_bias)
      , m_gain(i_gain) {
        m_xformInverted = m_xform;
        m_xformInverted.invert();
    }

    // Eval!
    T_t eval(const Vec3<T_t>& v, T_t t) const {
        Vec3<T_t> src;
        m_xformInverted.multVecMatrix(v, src);

        return m_bias + m_gain * m_pattern(src, t * m_evolveRate);
    }

    // 2D noise
    T_t operator()(T_t x, T_t y) const {
        return eval(Vec3<T_t>(x, y, 0.0), 0.0);
    }

    T_t operator()(const Vec2<T_t>& v) const {
        return eval(Vec3<T_t>(v.x, v.y, 0.0), 0.0);
    }

    // 3D noise
    T_t operator()(T_t x, T_t y, T_t z) const {
        return eval(Vec3<T_t>(x, y, z), 0.0);
    }

    T_t operator()(const Vec3<T_t>& v) const {
        return eval(v, 0.0);
    }

    // 4D noise.
    T_t operator()(const Vec3<T_t>& v, T_t t) const {
        return eval(v, t);
    }

    T_t operator()(T_t x, T_t y, T_t z, T_t w) const {
        return eval(Vec3<T_t>(x, y, z), w);
    }

protected:
    PATTERN m_pattern;
    matrix_type m_xform;
    matrix_type m_xformInverted;
    value_type m_evolveRate;
    value_type m_bias;
    value_type m_gain;
};

}  // End namespace noise
}  // End namespace emerald
