//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#ifndef _EmldCore_Util_Functions_h_
#define _EmldCore_Util_Functions_h_

//#include "Foundation.h"

#include <emerald/util/functions.h>

//#include <OpenEXR/ImathBox.h>
//#include <OpenEXR/ImathVec.h>

//#include <cmath>

namespace EmldCore {
namespace Util {

using namespace emerald::util;

#if 0
//-*****************************************************************************
//-*****************************************************************************
// SHOULDER FUNCTION - this implements a curve that's basically just 'x'
// for x <= 0.5, and then asymptotically approaches 1 for any value greater
// than 0.5. This can be used to implement a soft clamp.
template <typename T>
inline T shoulder(const T& x) {
    EMLD_LITERAL_CONSTANT T Half = 0.5;
    EMLD_LITERAL_CONSTANT T Two = 2.0;

    if (x <= Half) {
        return x;
    } else {
        return Half + (Half * std::tanh(Two * (x - Half)));
    }
}

//-*****************************************************************************
// Limit value to never get larger than maxVal, using shoulder curve.
template <typename T>
inline T shoulderLimit(const T& x, const T& maxVal) {
    return maxVal * shoulder(x / maxVal);
}
#endif

//-*****************************************************************************
// The Gamma functions (Euler's Gamma Function) are implemented in the .cpp.  //
// Description:                                                               //
//     The Gamma function of x for real x > 0 is defined as:                  //
//               Gamma(x) = Integral[0,inf] t^(x-1) exp(-t) dt                //
//     and analytically continued in the complex plane with simple poles at   //
//     the nonpositive integers, i.e. the Gamma function is a meromorphic     //
//     function with simple poles at the nonpositive integers.                //
//     For real x < 0, the Gamma function satisfies the reflection equation:  //
//                Gamma(x) = pi / ( sin(pi*x) * Gamma(1-x) ).                 //
//                                                                            //
//     The functions Gamma_Function() and xGamma_Function() return the Gamma  //
//     function evaluated at x for x real.                                    //
//                                                                            //
//     The function Gamma_Function_Max_Arg() returns the maximum argument of  //
//     the Gamma function for arguments > 1 and return values of type double. //
//                                                                            //
//     The function xGamma_Function_Max_Arg() returns the maximum argument of //
//     the Gamma function for arguments > 1 and return values of type long    //
//     double.                                                                //
//-*****************************************************************************
double Gamma_Function(double x);
long double xGamma_Function(long double x);
double Gamma_Function_Max_Arg(void);
long double xGamma_Function_Max_Arg(void);

#if 0

//-*****************************************************************************
// A collection of simple functions borrowed from various places
// to make this library self-sufficient.
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
inline const T& clamp(const T& val, const T& lo, const T& hi) {
    return val < lo ? lo : val > hi ? hi : val;
}

//-*****************************************************************************
// You'd think instead of doing the a*(1-t) + b*t, it'd be faster
// and one less multiply to do a + (b-a)*t, right? Bad! Increases floating
// point exception occurances. Same as LERP
template <class T, class T2>
inline T mix(const T& a, const T& b, const T2& interp) {
    EMLD_LITERAL_CONSTANT T2 one = ((T2)1);
    return (a * (one - interp)) + (b * interp);
}

//-*****************************************************************************
template <class T>
inline const T& sign(const T& val) {
    EMLD_LITERAL_CONSTANT T positive = ((T)1);
    EMLD_LITERAL_CONSTANT T negative = ((T)-1);
    EMLD_LITERAL_CONSTANT T zero = ((T)0);
    return val < zero ? negative : val > zero ? positive : zero;
}

//******************************************************************************
template <class T>
inline T radians(const T& deg) {
    EMLD_LITERAL_CONSTANT T TPI = ((T)M_PI);
    EMLD_LITERAL_CONSTANT T ONEEIGHTY = ((T)180);
    return TPI * (deg / ONEEIGHTY);
}

//******************************************************************************
template <class T>
inline T degrees(const T& rad) {
    EMLD_LITERAL_CONSTANT T TPI = ((T)M_PI);
    EMLD_LITERAL_CONSTANT T ONEEIGHTY = ((T)180);
    return ONEEIGHTY * (rad / TPI);
}

//-*****************************************************************************
// f(x) = c0 + c1*x + c2*x^2 + c3*x^3
template <typename T1, typename T2>
inline T1 cubic(const T1& coeff0,
                const T1& coeff1,
                const T1& coeff2,
                const T1& coeff3,
                const T2& t) {
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
inline T1 hermite(const T1& pointA,
                  const T1& pointB,
                  const T1& slopeA,
                  const T1& slopeB,
                  const T2& t) {
    T1 M = pointB - pointA - slopeA;
    T1 M2 = M * ((T2)2);
    T1 N = slopeB - slopeA;
    T1 c3 = N - M2;
    return cubic(pointA, slopeA, M - c3, c3, t);
}

//******************************************************************************
// Smoothstep function
// Goes from 0 to 1.
template <class T>
inline T smoothstep(const T& t) {
    if (t <= (T)0) {
        return (T)0;
    } else if (t >= (T)1) {
        return (T)1;
    } else {
        return t * t * ((T)3 - (t * (2)));
    }
}

//******************************************************************************
template <class T>
inline T smoothstep(const T& edge0, const T& edge1, const T& t) {
    return smoothstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
template <class T>
inline const T& linstep(const T& t) {
    EMLD_LITERAL_CONSTANT T t0 = ((T)0);
    EMLD_LITERAL_CONSTANT T t1 = ((T)1);

    return clamp(t, t0, t1);
}

//-*****************************************************************************
template <class T>
inline T linstep(const T& edge0, const T& edge1, const T& t) {
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
typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value,
                        T>::type
wrap(T i_x, T i_lowerBound, T i_upperBound) {
    const T rangeSize = i_upperBound - i_lowerBound + 1;

    if (i_x < i_lowerBound) {
        i_x += rangeSize * (((i_lowerBound - i_x) / rangeSize) + 1);
    }

    return i_lowerBound + (i_x - i_lowerBound) % rangeSize;
}

//-*****************************************************************************
template <typename T>
typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value,
                        T>::type
wrap(T i_x, T N) {
    if (i_x < 0) { i_x += N * (((-i_x) / N) + 1); }

    return i_x % N;
}

//-*****************************************************************************
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type wrap(T x,
                                                                        T n) {
    return x - (n * std::floor(x / n));
}

//-*****************************************************************************
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type wrap(T x,
                                                                        T lb,
                                                                        T ub) {
    return lb + wrap(x - lb, ub - lb);
}

//-*****************************************************************************
// END WRAP
//-*****************************************************************************

//-*****************************************************************************
template <class T>
inline T sqr(const T& a) {
    return a * a;
}

//-*****************************************************************************
template <typename T>
inline T cube(const T& a) {
    return a * a * a;
}

//-*****************************************************************************
//-*****************************************************************************
// BOX INTERSECTION STUFF
//-*****************************************************************************
//-*****************************************************************************

template <typename T>
inline Imath::Box<Imath::Vec3<T> > BoxIntersection(
  const Imath::Box<Imath::Vec3<T> >& i_a,
  const Imath::Box<Imath::Vec3<T> >& i_b) {
    return Imath::Box<Imath::Vec3<T> >(
      Imath::Vec3<T>(std::max(i_a.min.x, i_b.min.x),
                     std::max(i_a.min.y, i_b.min.y),
                     std::max(i_a.min.z, i_b.min.z)),

      Imath::Vec3<T>(std::min(i_a.max.x, i_b.max.x),
                     std::min(i_a.max.y, i_b.max.y),
                     std::min(i_a.max.z, i_b.max.z)));
}

template <typename T>
inline Imath::Box<Imath::Vec3<T> > BoxIntersection(
  const Imath::Box<Imath::Vec3<T> >& i_a,
  const Imath::Box<Imath::Vec3<T> >& i_b,
  const Imath::Box<Imath::Vec3<T> >& i_c) {
    return BoxIntersection<T>(BoxIntersection<T>(i_a, i_b), i_c);
}
#endif

}  // End namespace Util
}  // End namespace EmldCore

#endif
