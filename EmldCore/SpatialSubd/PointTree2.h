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

#ifndef _EmldCore_SpatialSubd_PointTree2_h_
#define _EmldCore_SpatialSubd_PointTree2_h_

#include "Foundation.h"
#include "KdTree2.h"
#include "Inclusion.h"

namespace EmldCore {
namespace SpatialSubd {

//-*****************************************************************************
// Bounds and Sort Point from a point (identity)
template <class T>
struct PointSortAndBounds2
{
    PointSortAndBounds2(){}
    const Imath::Vec2<T> &operator()( const Imath::Vec2<T> &p ) const
    {
        return p;
    }
};

//-*****************************************************************************
// Closest Point Traverser
template <class T>
struct ClosestPointTraverser2
{
    ClosestPointTraverser2( const Imath::Vec2<T> &p )
      : testPoint( p ),
        bestSquaredDist( std::numeric_limits<T>::max() ),
        maxSquaredDist( std::numeric_limits<T>::max() ),
        bestPoint( p ) {}

    ClosestPointTraverser2( const Imath::Vec2<T> &p,
                            const T &maxD2 )
      : testPoint( p ),
        bestSquaredDist( maxD2 ),
        maxSquaredDist( maxD2 ),
        bestPoint( p ) {}
    
    bool validBounds( const Imath::Box<Imath::Vec2<T> > &bnds )
    {
        return PointBoxMinimumSquaredDistanceLessThan( testPoint, bnds,
                                                       bestSquaredDist );
    }
    
    void leaf( const Imath::Vec2<T> &point )
    {
        T d2 = ( point - testPoint ).length2();
        if ( d2 < bestSquaredDist )
        {
            bestSquaredDist = d2;
            bestPoint = point;
        }
    }

    // Always need to keep checking for closest point stuff.
    bool unfinished() const { return true; }

    bool foundAny() const
    {
        return bestSquaredDist < maxSquaredDist;
    }

    Imath::Vec2<T> testPoint;
    T bestSquaredDist;
    T maxSquaredDist;
    Imath::Vec2<T> bestPoint;
};  

//-*****************************************************************************
// Don't derive from PointTree2. Just Typedef it.
template <class T>
class PointTree2 : public KdTree2<T,
                                  Imath::Vec2<T>,
                                  PointSortAndBounds2<T>,
                                  PointSortAndBounds2<T> >
{
public:
    PointTree2( Imath::Vec2<T> *Begin,
                Imath::Vec2<T> *End,
                int maxPerLeaf = 8, int maxSubDivs = 16 )
      : KdTree2<T,
                Imath::Vec2<T>,
                PointSortAndBounds2<T>,
                PointSortAndBounds2<T> >( Begin, End,
                                          PointSortAndBounds2<T>(),
                                          PointSortAndBounds2<T>(),
                                          maxPerLeaf,
                                          maxSubDivs )
    {
        // Nothing;
    }
    
    PointTree2( std::vector<Imath::Vec2<T> > &pts,
                int maxPerLeaf = 8, int maxSubDivs = 16 )
      : KdTree2<T,
                Imath::Vec2<T>,
                PointSortAndBounds2<T>,
                PointSortAndBounds2<T> >( pts,
                                          PointSortAndBounds2<T>(),
                                          PointSortAndBounds2<T>(),
                                          maxPerLeaf,
                                          maxSubDivs )
    {
        // Nothing;
    }

    // Return the closest point to a test point by reference, return
    // whether any were found.
    bool closestPoint( const Imath::Vec2<T> &testPoint,
                       Imath::Vec2<T> &bestPoint,
                       T &bestSquaredDist ) const
    {
        ClosestPointTraverser2<T> cpt( testPoint );
        this->constTraverse( cpt );
        if ( cpt.foundAny() )
        {
            bestPoint = cpt.bestPoint;
            bestSquaredDist = cpt.bestSquaredDist;
            return true;
        }
        return false;
    }

    // Return the closest point to a test point within some max squared
    // max distance.
    bool closestPointWithinSquaredDistance( const Imath::Vec2<T> &testPoint,
                                            const T &maxSquaredDist,
                                            Imath::Vec2<T> &bestPoint,
                                            T &bestSquaredDist ) const
    {
        ClosestPointTraverser2<T> cpt( testPoint, maxSquaredDist );
        this->constTraverse( cpt );
        if ( cpt.foundAny() )
        {
            bestPoint = cpt.bestPoint;
            bestSquaredDist = cpt.bestSquaredDist;
            return true;
        }
        return false;
    }
    
};

//-*****************************************************************************
typedef PointTree2<float> PointTree2f;
typedef PointTree2<double> PointTree2d;


} // End namespace SpatialSubd
} // End namespace EmldCore

#endif
