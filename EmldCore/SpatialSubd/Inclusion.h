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

#ifndef _EmldCore_SpatialSubd_Inclusion_h_
#define _EmldCore_SpatialSubd_Inclusion_h_

#include "Foundation.h"

#include <OpenEXR/ImathFun.h>
#include <OpenEXR/ImathInterval.h>

#include <algorithm>
#include <utility>

namespace EmldCore {
namespace SpatialSubd {

//-*****************************************************************************
// A common thing we need to do with spatial subdivision is ask whether
// the smallest squared distance between some point and any points
// in a bounding box is smaller than some minimum squared distance.
//
// The two functions descibed here perform Interval Arithmetic to answer
// these questions as efficiently as possible.
//
// This one returns the inclusion (range) of squared distances from a point
// to a range of points (box)
//
// template <class T>
// Imath::Interval<T> PointBoxSquaredDistanceInclusion
// ( const Imath::Vec2<T> &point, const Imath::Box<Imath::Vec2<T> > &box );
//
// template <class T>
// Imath::Interval<T> PointBoxSquaredDistanceInclusion
// ( const Imath::Vec3<T> &point, const Imath::Box<Imath::Vec3<T> > &box );
//
// This one answers whether the squared distance between a point and any
// point in a range of points (box) is possibly less than some minimum
// squared distance
//
// template <class T>
// bool PointBoxMinimumSquaredDistanceLessThan
// ( const Imath::Vec2<T> &point, const Imath::Box<Imath::Vec2<T> > &box,
//   const T &givenSquaredDist )
//
// template <class T>
// bool PointBoxMinimumSquaredDistanceLessThan
// ( const Imath::Vec3<T> &point, const Imath::Box<Imath::Vec3<T> > &box,
//   const T &givenSquaredDist )
//-*****************************************************************************

//-*****************************************************************************
// Use anonymous namespace to hide these functions from anything except this
// file.
namespace {

//-*****************************************************************************
// This function adds a range to an existing range.
template <class T>
inline void __incHelp(T bestDpMin, T bestDpMax, Imath::Interval<T>& rslt) {
    if (Imath::sign(bestDpMin) != Imath::sign(bestDpMax)) {
        bestDpMax = std::max(bestDpMin * bestDpMin, bestDpMax * bestDpMax);
        bestDpMin = ((T)0);
    } else {
        bestDpMax *= bestDpMax;
        bestDpMin *= bestDpMin;
        if (bestDpMin > bestDpMax) { std::swap(bestDpMin, bestDpMax); }
    }

    rslt.min += bestDpMin;
    rslt.max += bestDpMax;
}

//-*****************************************************************************
// Returns the minimum of the interval obtained by squaring the given
// interval.
template <class T>
inline T __minDist2(T bestDpMin, T bestDpMax) {
    if (Imath::sign(bestDpMin) != Imath::sign(bestDpMax)) {
        return ((T)0);
    } else {
        bestDpMax *= bestDpMax;
        bestDpMin *= bestDpMin;
        return (bestDpMin < bestDpMax) ? bestDpMin : bestDpMax;
    }
}

//-*****************************************************************************
// Returns the maximum of the interval obtained by squaring the given
// interval.
template <class T>
inline T __maxDist2(T bestDpMin, T bestDpMax) {
    bestDpMin *= bestDpMin;
    bestDpMax *= bestDpMax;
    return (bestDpMin < bestDpMax) ? bestDpMax : bestDpMin;
}

}  // End anonymous namespace

//-*****************************************************************************
// This is an "inclusion function", in the pure interval arithmetic
// sense of the word. It returns, for a 3D point and a Box, what
// the range of possible values are for the squared distance between the point
// and ANY point that might lie in the box. It attempts to do this as
// quickly as possible.

//-*****************************************************************************
template <class T>
inline Imath::Interval<T> PointBoxSquaredDistanceInclusion(
  const Imath::Vec2<T>& point, const Imath::Box<Imath::Vec2<T> >& range) {
    Imath::Interval<T> rslt(((T)0), ((T)0));

    __incHelp(point.x - range.max.x, point.x - range.min.x, rslt);
    __incHelp(point.y - range.max.y, point.y - range.min.y, rslt);

    return rslt;
}

//-*****************************************************************************
template <class T>
inline Imath::Interval<T> PointBoxSquaredDistanceInclusion(
  const Imath::Vec3<T>& point, const Imath::Box<Imath::Vec3<T> >& range) {
    Imath::Interval<T> rslt(((T)0), ((T)0));

    __incHelp(point.x - range.max.x, point.x - range.min.x, rslt);
    __incHelp(point.y - range.max.y, point.y - range.min.y, rslt);
    __incHelp(point.z - range.max.z, point.z - range.min.z, rslt);

    return rslt;
}

//-*****************************************************************************
// This one's even trimmer, attempts to find out whether the squared
// distance between a point and any point within a bounds could possibly
// be less than some given value.
template <class T>
inline bool PointBoxMinimumSquaredDistanceLessThan(
  const Imath::Vec2<T>& point, const Imath::Box<Imath::Vec2<T> >& range, T d2) {
    T md2 = __minDist2(point.x - range.max.x, point.x - range.min.x);
    if (md2 >= d2) { return false; }
    md2 += __minDist2(point.y - range.max.y, point.y - range.min.y);
    return (md2 < d2);
}

//-*****************************************************************************
template <class T>
inline bool PointBoxMinimumSquaredDistanceLessThan(
  const Imath::Vec3<T>& point, const Imath::Box<Imath::Vec3<T> >& range, T d2) {
    T md2 = __minDist2(point.x - range.max.x, point.x - range.min.x);
    if (md2 >= d2) { return false; }
    md2 += __minDist2(point.y - range.max.y, point.y - range.min.y);
    if (md2 >= d2) { return false; }
    md2 += __minDist2(point.z - range.max.z, point.z - range.min.z);
    return (md2 < d2);
}

}  // End namespace SpatialSubd
}  // End namespace EmldCore

#endif
