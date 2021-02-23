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

#include <EmldCore/CompactHashMap/BaseCompactHashMap.h>
#include <EmldCore/CompactHashMap/VectorManager.h>
#include <EmldCore/CompactHashMap/IndexBlock.h>

#include <EmldCore/ParallelUtil/All.h>
#include <EmldCore/Util/All.h>

using namespace EmldCore::CompactHashMap;
using namespace EmldCore::ParallelUtil;
using namespace EmldCore::Util;

typedef IndexBlock<V2i,int,96> cell_type;
typedef BucketedConcurrentVector<V2i> coord_vec;
typedef BucketedConcurrentVector<cell_type> cell_vec;

//-*****************************************************************************
struct CellWithPoints
{
    V2i coord;
    int numPoints;
    CellWithPoints( const V2i& i_c, int i_np ) 
    : coord( i_c ), numPoints( i_np ) {}

    template <typename COORD_VEC>
    void emit( COORD_VEC& o_coords )
    {
        for ( int i = 0; i < numPoints; ++i )
        {
            o_coords.push_back( coord );
        }
    }
};

//-*****************************************************************************
void TestWithoutVmgr()
{
    EMLD_ASSERT( sizeof( cell_type ) == 96, "Bad size." );

    typedef BaseCompactHashMap<cell_type,cell_vec> bchm_type;

    std::vector<CellWithPoints> cells;
    cells.push_back( CellWithPoints( V2i( -17, 82 ), 14 ) );
    cells.push_back( CellWithPoints( V2i( -16, 82 ), 11 ) );
    cells.push_back( CellWithPoints( V2i( -15, 82 ), 9 ) );
    cells.push_back( CellWithPoints( V2i( -14, 82 ), 23 ) );
    cells.push_back( CellWithPoints( V2i( -18, 83 ), 17 ) );
    cells.push_back( CellWithPoints( V2i( -17, 83 ), 15 ) );
    cells.push_back( CellWithPoints( V2i( -16, 83 ), 3 ) );
    cells.push_back( CellWithPoints( V2i( -15, 83 ), 2 ) );
    cells.push_back( CellWithPoints( V2i( -12, 84 ), 31 ) );
    cells.push_back( CellWithPoints( V2i( -11, 84 ), 19 ) );
    cells.push_back( CellWithPoints( V2i( -10, 84 ), 13 ) );
    int numCells = cells.size();
    std::cout << "Made " << numCells << " dummy cells." << std::endl;

    coord_vec coords;
    for ( std::vector<CellWithPoints>::iterator iter = cells.begin();
          iter != cells.end(); ++iter )
    {
        (*iter).emit( coords );
    }
    std::cout << "Emitted coordinates." << std::endl;

    // Assert sorted.
    int numCoords = coords.size();
    std::cout << "Num coords: " << numCoords << std::endl;
    for ( int i = 1; i < numCoords; ++i )
    {
        EMLD_ASSERT( coords[i-1] <= coords[i], "Out of order" );
    }

    // Rebuild the hash map from them.
    bchm_type BCHM;
    {
        std::vector<int> tmpIndices;
        BCHM.rebuild( coords, tmpIndices );
    }
    std::cout << "Rebuilt base chm." << std::endl;
    EMLD_ASSERT( BCHM.size() == numCells, "Wrong number of cells." );

    // Verify that each cell has the indices we'd expect it to.
    int coordIndex = 0;
    for ( int cell = 0; cell < numCells; ++cell )
    {
        const CellWithPoints &cp = cells[cell];
        const cell_type* hcell = BCHM.find( cp.coord );
        EMLD_ASSERT( hcell != NULL, "Can't find a cell we should have." );
        for ( int p = 0; p < cp.numPoints; ++p, ++coordIndex )
        {
            EMLD_ASSERT( hcell->contains( coordIndex ),
                            "Cell is missing coordinate it should have." );
        }
    }
    std::cout << "Validated that each cell had all the indices it should."
              << std::endl;

    BCHM.debugPrintOccupancyRatio();
    BCHM.debugValidateElementIndexRange( 0, numCoords, coords );
}

//-*****************************************************************************
int main( int argc, char *argv[] )
{
    TestWithoutVmgr();
}

