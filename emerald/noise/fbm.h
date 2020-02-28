#pragma once

#include <emerald/noise/foundation.h>
#include <emerald/noise/pattern_gradient.h>
#include <emerald/noise/simplex_noise.h>

namespace emerald {
namespace noise {

//-*****************************************************************************
// This is a templated function which takes a noise functor, like the
// SimplexNoise one at the end of the SimplexNoise.h file, and uses it to
// create a recursive pattern.
// It relies on the "value_type" and "output_type" typedefs on the pattern
// (see SimplexNoise), but nothing else.
// This would allow for an alternative output scheme.
// For example, if we created a GradientNoise3 functor.

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// 2D STUFF
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class PATTERN>
typename PATTERN::output_type fbm(PATTERN& pattern,
                                  typename PATTERN::value_type x,
                                  typename PATTERN::value_type y,
                                  int octaves,
                                  typename PATTERN::value_type lacunarity,
                                  typename PATTERN::value_type octaveGain) {
    // x & y are passed by value, we can modify them.
    typename PATTERN::output_type sum = pattern(x, y);
    typename PATTERN::value_type amp = (typename PATTERN::value_type)1.0;

    for (int oct = 1; oct < octaves; ++oct) {
        amp *= octaveGain;
        x *= lacunarity;
        y *= lacunarity;

        sum += amp * pattern(x, y);
    }

    return sum;
}

//-*****************************************************************************
template <class PATTERN>
typename PATTERN::output_type fbm(
    PATTERN& pattern,
    const Imath::Vec2<typename PATTERN::value_type>& p,
    int octaves,
    typename PATTERN::value_type lacunarity,
    typename PATTERN::value_type octaveGain) {
    typename PATTERN::output_type sum = pattern(p);
    typename PATTERN::value_type amp = (typename PATTERN::value_type)1.0;
    Imath::Vec2<typename PATTERN::value_type> pp = p;

    for (int oct = 1; oct < octaves; ++oct) {
        amp *= octaveGain;
        pp *= lacunarity;
        sum += amp * pattern(pp);
    }

    return sum;
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// 3D STUFF
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class PATTERN>
typename PATTERN::output_type fbm(PATTERN& pattern,
                                  typename PATTERN::value_type x,
                                  typename PATTERN::value_type y,
                                  typename PATTERN::value_type z,
                                  int octaves,
                                  typename PATTERN::value_type lacunarity,
                                  typename PATTERN::value_type octaveGain) {
    // x, y, and z are passed by value, we can modify them.
    typename PATTERN::output_type sum = pattern(x, y, z);
    typename PATTERN::value_type amp = (typename PATTERN::value_type)1.0;

    for (int oct = 1; oct < octaves; ++oct) {
        amp *= octaveGain;
        x *= lacunarity;
        y *= lacunarity;
        z *= lacunarity;

        sum += amp * pattern(x, y, z);
    }

    return sum;
}

//-*****************************************************************************
template <class PATTERN>
typename PATTERN::output_type fbm(
    PATTERN& pattern,
    const Imath::Vec3<typename PATTERN::value_type>& p,
    int octaves,
    typename PATTERN::value_type lacunarity,
    typename PATTERN::value_type octaveGain) {
    typename PATTERN::output_type sum = pattern(p);
    typename PATTERN::value_type amp = (typename PATTERN::value_type)1.0;
    Imath::Vec3<typename PATTERN::value_type> pp = p;

    for (int oct = 1; oct < octaves; ++oct) {
        amp *= octaveGain;
        pp *= lacunarity;
        sum += amp * pattern(pp);
    }

    return sum;
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// 4D STUFF
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class PATTERN>
typename PATTERN::output_type fbm(PATTERN& pattern,
                                  typename PATTERN::value_type x,
                                  typename PATTERN::value_type y,
                                  typename PATTERN::value_type z,
                                  typename PATTERN::value_type w,
                                  int octaves,
                                  typename PATTERN::value_type lacunarity,
                                  typename PATTERN::value_type octaveGain) {
    // x, y, z  & w are passed by value, we can modify them.
    typename PATTERN::output_type sum = pattern(x, y, z, w);
    typename PATTERN::value_type amp = (typename PATTERN::value_type)1.0;

    for (int oct = 1; oct < octaves; ++oct) {
        amp *= octaveGain;
        x *= lacunarity;
        y *= lacunarity;
        z *= lacunarity;
        w *= lacunarity;

        sum += amp * pattern(x, y, z, w);
    }

    return sum;
}

//-*****************************************************************************
template <class PATTERN>
typename PATTERN::output_type fbm(
    PATTERN& pattern,
    const Imath::Vec3<typename PATTERN::value_type>& p,
    typename PATTERN::value_type t,
    int octaves,
    typename PATTERN::value_type lacunarity,
    typename PATTERN::value_type octaveGain) {
    typename PATTERN::output_type sum = pattern(p, t);
    typename PATTERN::value_type amp = (typename PATTERN::value_type)1.0;
    Imath::Vec3<typename PATTERN::value_type> pp = p;

    for (int oct = 1; oct < octaves; ++oct) {
        amp *= octaveGain;
        pp *= lacunarity;
        t *= lacunarity;
        sum += amp * pattern(pp, t);
    }

    return sum;
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// In Functor Format.
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class PATTERN>
class Fbm {
public:
    typedef PATTERN pattern_type;
    typedef typename PATTERN::value_type value_type;
    typedef typename PATTERN::value_type T_t;
    typedef typename PATTERN::output_type output_type;
    typedef Imath::Vec2<T_t> point_type_2d;
    typedef Imath::Vec2<T_t> point_type_3d;
    typedef Fbm<PATTERN> this_type;

    Fbm(PATTERN pattern, int octaves, float lacunarity, float octGain)
      : m_pattern(pattern)
      , m_octaves(octaves)
      , m_lacunarity(lacunarity)
      , m_octaveGain(octGain) {
    }

    // 2D noise
    output_type operator()(T_t x, T_t y) const {
        return fbm(m_pattern, x, y, m_octaves, m_lacunarity, m_octaveGain);
    }

    output_type operator()(const Imath::Vec2<T_t>& p) const {
        return fbm(m_pattern, p, m_octaves, m_lacunarity, m_octaveGain);
    }

    // 3D noise
    output_type operator()(T_t x, T_t y, T_t z) const {
        return fbm(m_pattern, x, y, z, m_octaves, m_lacunarity, m_octaveGain);
    }

    output_type operator()(const Imath::Vec3<T_t>& p) const {
        return fbm(m_pattern, p, m_octaves, m_lacunarity, m_octaveGain);
    }

    // 4D noise
    output_type operator()(T_t x, T_t y, T_t z, T_t w) const {
        return fbm(
            m_pattern, x, y, z, w, m_octaves, m_lacunarity, m_octaveGain);
    }

    output_type operator()(const Imath::Vec3<T_t>& p, T_t t) const {
        return fbm(m_pattern, p, t, m_octaves, m_lacunarity, m_octaveGain);
    }

protected:
    PATTERN m_pattern;
    int m_octaves;
    float m_lacunarity;
    float m_octaveGain;
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// Instantiations for Simplex Noise.
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
template <class T>
class StdFbm : public Fbm<SimplexNoise<T> > {
public:
    StdFbm(int oct, T lac, T og)
      : Fbm<SimplexNoise<T> >(SimplexNoise<T>(), oct, lac, og) {
    }
};

typedef StdFbm<float> StdFbmf;
typedef StdFbm<double> StdFbmd;

}  // End namespace noise
}  // End namespace emerald
