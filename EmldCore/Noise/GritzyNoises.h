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

#ifndef _EmldCore_Noise_GritzyNoises_h_
#define _EmldCore_Noise_GritzyNoises_h_

#include "Foundation.h"
#include "SimplexNoise.h"
#include "Fbm.h"
#include "CellNoise.h"

namespace EmldCore {
namespace Noise {

/***************************************************************************
 * Voronoi cell noise (a.k.a. Worley noise) functions
 *
 * These functions assume that space is filled with "features" (points
 * of interest).  There are interestingpatterns we can make by
 * figuring out which feature we are closest to, or to what extent
 * we're on the boundary between two features.  Several varieties of
 * these computations are below, categorized by the dimension of their
 * domains, and the number of close features they are interested in.
 *
 * All these functions have similar inputs:
 *   P      - position to test (for 3-D varieties; 2-D varieties use ss,tt)
 *   jitter - how much to jitter the cell center positions (1 is typical,
 *             smaller values make a more regular pattern, larger values
 *             make a more jagged pattern; use jitter >1 at your risk!).
 * And outputs:
 *   f_n    - distance to the nth nearest feature (f1 is closest, f2 is
 *            the distance to the 2nd closest, etc.)
 *   pos_n  - the position of the nth nearest feature.  For 2-D varieties,
 *            these are instead spos_n and tpos_n.
 ***************************************************************************/

//-*****************************************************************************
template <typename T>
void voronoi_f1f2( const Imath::Vec3<T> &iPoint,
                   T iJitter,
                   T &oF1, Imath::Vec3<T> &oPos1,
                   T &oF2, Imath::Vec3<T> &oPos2 )
{
    typedef Imath::Vec3<T> Point;
    using namespace Imath;
    
    Point thisCell( Math<T>::floor( iPoint.x + 0.5 ),
                    Math<T>::floor( iPoint.y + 0.5 ),
                    Math<T>::floor( iPoint.z + 0.5 ) );
    
    oF1 = 1000.0;
    oF2 = 1000.0;
    for ( int i = -1;  i <= 1;  ++i )
    {
        for ( int j = -1;  j <= 1;  ++j )
        {
            for ( int k = -1;  k <= 1;  ++k )
            {
                Point testCell = thisCell + Point( i, j, k );
                Point pos = testCell + 
                    ((T)0.5) * iJitter * CellNoise3( testCell );
		Point offset = pos - iPoint;
                T dist2 = offset.length2();
                if ( dist2 < oF1 )
                {
                    oF2 = oF1;
                    oPos2 = oPos1;
                    oF1 = dist2;
                    oPos1 = pos;
                }
                else if ( dist2 < oF2 )
                {
                    oF2 = dist2;
                    oPos2 = pos;
		}
            }
	}
    }
    oF1 = Math<T>::sqrt( oF1 );
    oF2 = Math<T>::sqrt( oF2 );
}


} // End namespace Noise
} // End namespace EmldCore

#endif
