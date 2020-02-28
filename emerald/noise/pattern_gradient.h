#pragma once

#include <emerald/noise/foundation.h>
#include <emerald/noise/simplex_noise.h>

namespace emerald {
namespace noise {

//-*****************************************************************************
template <class PATTERN>
class PatternGradient2D {
public:
    typedef PATTERN pattern_type;
    typedef typename PATTERN::value_type value_type;
    typedef typename PATTERN::value_type T_t;
    typedef Imath::Vec2<T_t> output_type;
    typedef PatternGradient2D<PATTERN> this_type;

    PatternGradient2D(PATTERN pattern)
      : m_pattern(pattern) {
    }

    // 2D Gradient
    output_type operator()(T_t x, T_t y) const {
        static const T_t tiny = ((T_t)0.01);

        T_t v0 = m_pattern(x, y);
        T_t vx = m_pattern(x + tiny, y);
        T_t vy = m_pattern(x, y + tiny);
        return Imath::Vec2<T_t>((vx - v0) / tiny, (vy - v0) / tiny);
    }

    output_type operator()(const Imath::Vec2<T_t>& p) const {
        return operator()(p.x, p.y);
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
typedef PatternGradient2D<SimplexNoisef> Vnoise2f;
typedef PatternGradient2D<SimplexNoised> Vnoise2d;

//-*****************************************************************************
template <class PATTERN>
class PatternGradient3D {
public:
    typedef PATTERN pattern_type;
    typedef typename PATTERN::value_type value_type;
    typedef typename PATTERN::value_type T_t;
    typedef Imath::Vec3<T_t> output_type;
    typedef PatternGradient3D<PATTERN> this_type;

    PatternGradient3D(PATTERN pattern)
      : m_pattern(pattern) {
    }

    // 3D Gradient
    output_type operator()(T_t x, T_t y, T_t z) const {
        static const T_t tiny = ((T_t)0.01);

        T_t v0 = m_pattern(x, y, z);
        T_t vx = m_pattern(x + tiny, y, z);
        T_t vy = m_pattern(x, y + tiny, z);
        T_t vz = m_pattern(x, y, z + tiny);
        return Imath::Vec3<T_t>(
            (vx - v0) / tiny, (vy - v0) / tiny, (vz - v0) / tiny);
    }

    output_type operator()(const Imath::Vec3<T_t>& p) const {
        return operator()(p.x, p.y, p.z);
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
typedef PatternGradient3D<SimplexNoisef> Vnoise3f;
typedef PatternGradient3D<SimplexNoised> Vnoise3d;

//-*****************************************************************************
template <class PATTERN>
class EvolvingPatternGradient3D {
public:
    typedef PATTERN pattern_type;
    typedef typename PATTERN::value_type value_type;
    typedef typename PATTERN::value_type T_t;
    typedef Imath::Vec3<T_t> output_type;
    typedef PatternGradient3D<PATTERN> this_type;

    EvolvingPatternGradient3D(PATTERN pattern)
      : m_pattern(pattern) {
    }

    // 3D Gradient
    output_type operator()(T_t x, T_t y, T_t z, T_t t) const {
        static const T_t tiny = ((T_t)0.01);

        T_t v0 = m_pattern(x, y, z, t);
        T_t vx = m_pattern(x + tiny, y, z, t);
        T_t vy = m_pattern(x, y + tiny, z, t);
        T_t vz = m_pattern(x, y, z + tiny, t);
        return Imath::Vec3<T_t>(
            (vx - v0) / tiny, (vy - v0) / tiny, (vz - v0) / tiny);
    }

    output_type operator()(const Imath::Vec3<T_t>& p, T_t t) const {
        return operator()(p.x, p.y, p.z, t);
    }

protected:
    PATTERN m_pattern;
};

//-*****************************************************************************
typedef EvolvingPatternGradient3D<SimplexNoisef> Evnoise3f;
typedef EvolvingPatternGradient3D<SimplexNoised> Evnoise3d;

}  // End namespace noise
}  // End namespace emerald
