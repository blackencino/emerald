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

#include <EmldCore/SpatialSubd/All.h>
#include <boost/random.hpp>
#include <map>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using Imath::V3i;
using Imath::V3f;
using Imath::Box3i;

struct Comparator
{
    bool operator()( const V3i &iA, const V3i &iB ) const
    {
        if ( iA.z < iB.z ) { return true; }
        else if ( iA.z > iB.z ) { return false; }
        else
        {
            if ( iA.y < iB.y ) { return true; }
            else if ( iA.y > iB.y ) { return false; }
            else
            {
                return iA.x < iB.x;
            }
        }
    }
};

typedef std::map<V3i,int,Comparator> V3iIntMap;
typedef EmldCore::SpatialSubd::DUOctree<int> V3iIntTree;

int main( int argc, char *argv[] )
{
    boost::mt19937 gen;
    boost::uniform_int<> dist( 1700, 1900 );
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> >
        die( gen, dist );

    V3iIntMap iMap;
    V3iIntTree iTree;

    for ( int j = 0; j < 5000; ++j )
    {
        V3i next( die(), die(), die() );
        iMap[next] = j;
        iTree.set( next, j );
    }

    for ( V3iIntMap::iterator iter = iMap.begin();
          iter != iMap.end(); ++iter )
    {
        const V3i &point = (*iter).first;
        int val = (*iter).second;
        int treeVal = 0;
        bool found = iTree.get( point, treeVal );

        if ( !found || val != treeVal )
        {
            std::cerr << "ERROR: Bad tree. " << std::endl
                      << "point: " << point << std::endl
                      << "val: " << val << std::endl
                      << "treeVal: " << treeVal << std::endl
                      << "found: " << found << std::endl;
            exit( -1 );
        }

        std::cout << "\tpoint: " << point
                  << "\tval: " << val
                  << "\ttreeVal: " << treeVal << std::endl;
    }

    std::cout << "All good." << std::endl;
        
    return 0;
}
