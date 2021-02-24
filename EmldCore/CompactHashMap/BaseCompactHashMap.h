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

#ifndef _EmldCore_CompactHashMap_BaseCompactHashMap_h_
#define _EmldCore_CompactHashMap_BaseCompactHashMap_h_

#include "Foundation.h"
#include "SpatialHash.h"
#include "IndexBlock.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
template <typename BLOCK,
          typename MANAGED_BLOCK_VECTOR = BucketedConcurrentVector<BLOCK> >
class BaseCompactHashMap
{
public:
    //-*************************************************************************
    // PUBLIC AND INHERITED TYPEDEFS
    //-*************************************************************************
    typedef BLOCK Cell;
    typedef typename BLOCK::coord_type cell_coord_type;
    typedef typename BLOCK::index_type index_type;
    typedef MANAGED_BLOCK_VECTOR managed_cell_vector_type;

    typedef std::size_t size_type;

    typedef BaseCompactHashMap<BLOCK, MANAGED_BLOCK_VECTOR> this_type;

    //-*************************************************************************
    // UNORDERED MAP TYPEDEF
    //-*************************************************************************
    typedef SpatialHashFunctor<cell_coord_type> cell_hash_type;
    typedef std::equal_to<cell_coord_type> cell_equal_type;
    

    // The unordered map typedef.
#if EMLD_USE_CXX11
    typedef std::unordered_map<cell_coord_type, index_type, 
                               cell_hash_type, cell_equal_type>
    InternalUnorderedMap;
#else
    typedef boost::unordered_map<cell_coord_type, index_type, 
                                 cell_hash_type, cell_equal_type>
    InternalUnorderedMap;
#endif

    //-*************************************************************************
    //-*************************************************************************
    // PROTECTED TYPES AND DATA
    //-*************************************************************************
    //-*************************************************************************
protected:
    // Overflow pool.
    typedef boost::object_pool<Cell> overflow_pool_type;
    typedef tbb::spin_mutex mutex_type;

    // Allocators and Deallocators
    // These are not intended to be called frequently in parallel - they
    // represent exception cases (overflow), and are protected by a hard
    // mutex.
    struct OverflowAlloc
    {
        overflow_pool_type* m_overflowPool;
        mutex_type& m_overflowMutex;

        OverflowAlloc( overflow_pool_type* i_pool,
                        mutex_type& i_mutex )
            : m_overflowPool( i_pool )
            , m_overflowMutex( i_mutex ) {}

        Cell* operator()( void ) 
        { 
            typename mutex_type::scoped_lock mlock( m_overflowMutex );
            return m_overflowPool->construct(); 
        }
    };

    struct OverflowDealloc
    {
        overflow_pool_type* m_overflowPool;
        mutex_type& m_overflowMutex;

        OverflowDealloc( overflow_pool_type* i_pool,
                         mutex_type& i_mutex )
            : m_overflowPool( i_pool )
            , m_overflowMutex( i_mutex ) {}

        void operator()( Cell* i_cell ) 
        {
            typename mutex_type::scoped_lock mlock( m_overflowMutex );
            m_overflowPool->destroy( i_cell ); 
        }
    };

    //-*************************************************************************
    // DATA
    //-*************************************************************************

    managed_cell_vector_type        m_cells;
    InternalUnorderedMap            m_unorderedMap;
    EMLD_UNIQUE_PTR<overflow_pool_type>              m_overflowPool;
    mutex_type                      m_overflowMutex;
    OverflowAlloc                   m_overflowAlloc;
    OverflowDealloc                 m_overflowDealloc;

    //-*************************************************************************
    // NONCOPYABLE
    //-*************************************************************************
private:
    BaseCompactHashMap( const this_type& i_copy );
    this_type& operator=( const this_type& i_copy );

    //-*************************************************************************
    //-*************************************************************************
    // PUBLIC FUNCTIONS
    //-*************************************************************************
    //-*************************************************************************

public:
    // Build an empty base compact hash map
    BaseCompactHashMap()
        : m_overflowPool( new overflow_pool_type )
        , m_overflowAlloc( m_overflowPool.get(), m_overflowMutex )
        , m_overflowDealloc( m_overflowPool.get(), m_overflowMutex )
    {
        // Nothing.
    }

    // Build a base compact hash map with an externally supplied managed 
    // block vector.
    BaseCompactHashMap( managed_cell_vector_type i_vec )
        : m_cells( i_vec )
        , m_overflowPool( new overflow_pool_type )
        , m_overflowAlloc( m_overflowPool.get(), m_overflowMutex )
        , m_overflowDealloc( m_overflowPool.get(), m_overflowMutex )
    {
        // Nothing
    }

    // A clear function
    void clear()
    {
        typename mutex_type::scoped_lock mlock( m_overflowMutex );

        m_unorderedMap.clear();
        m_cells.clear();
        m_overflowPool.reset();
        m_overflowPool.reset( new overflow_pool_type );
        m_overflowAlloc.m_overflowPool = m_overflowPool.get();
        m_overflowDealloc.m_overflowPool = m_overflowPool.get();
    }

    // Destroy.
    ~BaseCompactHashMap()
    {
        clear();
    }

    // Returns the number of spatial cells.
    size_type size() const { return m_cells.size(); }

    // Return a spatial cell by index.
    Cell& cell( index_type i_cellIndex )
    { return m_cells[i_cellIndex]; }
    const Cell& cell( index_type i_cellIndex ) const
    { return m_cells[i_cellIndex]; }

    // CAREFULLY.
    Cell* cellsData() { return vector_data( m_cells ); }
    const Cell* cellsData() const { return vector_cdata( m_cells ); }

    // Find a cell, and return NULL if not found.
    Cell* find( const cell_coord_type& i_cellCoord )
    {
        typename InternalUnorderedMap::iterator fiter =
            m_unorderedMap.find( i_cellCoord );
        if ( fiter == m_unorderedMap.end() )
        {
            return NULL;
        }
        else
        {
            return &( m_cells[( *fiter ).second ] );
        }
    }

    const Cell* find( const cell_coord_type& i_cellCoord ) const
    {
        // CJH CONCERN: Hopefully the find function is read-only.
        return ( const_cast<this_type*>( this ) )->find( i_cellCoord );
    }

    // Rebuild the compact hash map from scratch.
    // The positions and grids must be sorted, such that all of the
    // particles with the same grid coordinate are adjacent in memory.
    template <typename CELL_COORD_VEC, typename INDEX_VEC>
    void rebuild( const CELL_COORD_VEC& i_sortedNewCellCoords,
                  INDEX_VEC& tmp_indices );

    // Incrementally update compact hash map structure
    // based on assumption that only a small number of elements
    // change cells.
    // This is performed partially in parallel, partially serially.
    // Cells Coordinates in the newCellCoords have already been computed,
    // and the tuples contain the old cell coordinates, and old indices.
    template <typename TUPLE, typename CELL_COORD_VEC>
    void moveElements( const TUPLE* i_moveBegin,
                       const TUPLE* i_moveEnd,
                       const CELL_COORD_VEC& i_newCellCoords );

    // Remove an element.
    void removeElement( index_type i_elemIndex,
                        const cell_coord_type& i_oldCellCoord );

    // Add an element.
    void addElement( index_type i_elemIndex,
                     const cell_coord_type& i_newCellCoord );

    // Debug: print out the number of cells that are nearly full or 
    // overflowed.
    void debugPrintOccupancyRatio();

    // Debug: For a range of indices and a vector representing the cell coords,
    // verify that each index is found in the cell corresponding to the coord
    // at that index.
    template <typename CELL_COORD_VEC>
    void debugValidateElementIndexRange( index_type i_beginIndex,
                                         index_type i_endIndex,
                                         const CELL_COORD_VEC& i_cellCoords );

};

//-*****************************************************************************
//-*****************************************************************************
// IMPLEMENTATION
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename CELL>
struct CellSetBeginEnd
{
    typedef CELL block_type;
    typedef typename CELL::index_type index_type;

    void set_begin( block_type& o_block, index_type i_begin ) const
    {
        o_block.setBeginIndex( i_begin );
    }

    void set_end( block_type& o_block, index_type i_end ) const
    {
        o_block.setEndIndex( i_end );
    }
};

//-*****************************************************************************
template <typename CELL, typename BLOCK_ALLOCATOR>
struct CompleteCells 
: public ParallelUtil::ZeroForEachFunctorI<CompleteCells<CELL, 
                                            BLOCK_ALLOCATOR> >
{
    typedef CELL cell_type;
    typedef BLOCK_ALLOCATOR alloc_type;
    typedef typename CELL::coord_type cell_coord_type;
    typedef typename CELL::index_type index_type;

    cell_type* Cells;
    const cell_coord_type* Coords;

    alloc_type* Alloc;

    void operator()( index_type i ) const
    {
        cell_type& cell = const_cast<cell_type&>( Cells[i] );
        cell.completeFromBeginEndIndices( const_cast<alloc_type&>( *Alloc ) );
        cell.setCoord( Coords[ cell.first() ] );
    }
};

//-*****************************************************************************
// This assumes that the positions have been sorted such that all the points
// with the same cell value are contiguous. This work is performed by
// the ReorderArrays class.
template <typename BLOCK, typename MBV>
template <typename CELL_COORD_VEC, typename INDEX_VEC>
void BaseCompactHashMap<BLOCK,MBV>
::rebuild( const CELL_COORD_VEC& i_sortedCellCoords,
           INDEX_VEC& tmp_cellIndices )
{
    // Clear our existing stuff.
    clear();

    // Get N. If there are no points, don't build anything.
    const index_type N = i_sortedCellCoords.size();
    if ( N < 1 ) 
    {
        return;
    }

    // Compute the cell index from cell coordinates, using ParallelUtil.
    tmp_cellIndices.resize( N );
    const index_type lastBlockIndex =
    ParallelUtil::VectorContiguousBlockIndicesEqual( i_sortedCellCoords,
                                                     tmp_cellIndices );
    const index_type numCells = lastBlockIndex + 1;
    std::cout << "NUM CELLS FOUND: " << numCells << std::endl;
    EMLD_DEBUG_ASSERT( lastBlockIndex >= 0 && lastBlockIndex < N,
                          "Out of range invalid block indices." );

    // Resize the cells.
    m_cells.resize( numCells );

    // Set contiguous block begin/end.
    {
        CellSetBeginEnd<Cell> csbe;
        ParallelUtil::VectorContiguousBlockSetBeginEnd( tmp_cellIndices,
                                                        m_cells,
                                                        csbe );
    }

    // Complete the cells.
    {
        CompleteCells<Cell,OverflowAlloc> F;
        F.Cells = vector_data( m_cells );
        F.Coords = vector_cdata( i_sortedCellCoords );
        F.Alloc = &m_overflowAlloc;
        F.execute( numCells );
    }

    // Last thing to do is populate the hash table.
    // As per CompactHashMap paper, we use 2* num particles buckets.
    m_unorderedMap.clear();
    m_unorderedMap.rehash( N * 2 );

    // Have to insert serially.
    for ( index_type i = 0; i < numCells; ++i )
    {
        m_unorderedMap[ m_cells[i].coord() ] = i;
    }

    // DONE.
}


//-*****************************************************************************
//-*****************************************************************************
// STUFF FOR MOVING ELEMENTS
// CJH NOTE: In the context of a particle-system with positions and velocities,
// the 'moving' adjective is overloaded in this case. All of the particles are
// most likely moving in space, but we're here referring to particles which
// have changed cells. The structure we use to find these particles
// is called PosCellIndexZ3, which agnostically describes the data.
//-*****************************************************************************
//-*****************************************************************************


//-*****************************************************************************
// This function is mostly serial.
template <typename BLOCK, typename MBV>
template <typename TUPLE, typename CELL_COORD_VEC>
void BaseCompactHashMap<BLOCK,MBV>
::moveElements( const TUPLE* i_moveBegin,
                const TUPLE* i_moveEnd,
                const CELL_COORD_VEC& i_newCellCoords )
{
    // Okay, moving elements exist. Now we have to loop over them
    // serially and move them from their old cell to their new cell.
    const size_type numMovingElems = i_moveEnd - i_moveBegin;
    if ( numMovingElems < 1 ) { return; }

    // First we remove them from their old cells, to make space.
    for ( const TUPLE* meIter = i_moveBegin;
          meIter != i_moveEnd; ++meIter )
    {
        const TUPLE& movingElem = ( *meIter );

        const cell_coord_type newCellCoord = i_newCellCoords[movingElem.index];

        // It's rare, but don't move particles if they're in the same cell.
        if ( newCellCoord == movingElem.cell ) { continue; }

        // The moving element has the element index and the OLD cell coordinate.
        this->removeElement( movingElem.index, movingElem.cell );
    }

    // Then we add them to their new cells.
    for ( const TUPLE* meIter = i_moveBegin;
          meIter != i_moveEnd; ++meIter )
    {
        const TUPLE& movingElem = ( *meIter );
        const cell_coord_type newCellCoord = i_newCellCoords[movingElem.index];

        // It's rare, but don't move particles if they're in the same cell.
        if ( newCellCoord == movingElem.cell ) { continue; }

        // The moving element has the element index, and we've got the NEW
        // cell coordinate.
        this->addElement( movingElem.index, newCellCoord );
    }
}

//-*****************************************************************************
// Remove element indices. This is a one-by-one ordeal.
template <typename BLOCK, typename MBV>
void BaseCompactHashMap<BLOCK,MBV>
::removeElement( index_type i_elemIndex,
                 const cell_coord_type& i_oldCellCoord )
{
    // Find the previous spatial cell, which MUST exist.
    typename InternalUnorderedMap::iterator oldCellIter =
        m_unorderedMap.find( i_oldCellCoord );
    EMLD_DEBUG_ASSERT( oldCellIter != m_unorderedMap.end(),
                      "Elements must have been residing previously in a "
                      "spatial cell" );

    index_type oldCellIndex = ( *oldCellIter ).second;
    Cell& oldCell = m_cells[oldCellIndex];
    EMLD_DEBUG_ASSERT( oldCell.coord() == i_oldCellCoord,
                          "Corrupt hash map." );

    // Remove element from that old cell.
    // This will complain if the element isn't in there already.
    oldCell.removeIndex( i_elemIndex, m_overflowDealloc );

    // If the cell is empty, we have to delete it.
    // Deleting the cell involves swapping it out with the last
    // cell in the list. That means we have to change the map
    // to point to this new index.
    if ( oldCell.empty() )
    {
        // Delete this cell coordinate from the map.
        m_unorderedMap.erase( oldCellIter );
        EMLD_DEBUG_ASSERT( m_unorderedMap.find( i_oldCellCoord ) ==
                              m_unorderedMap.end(),
                              "Bad unordered map erase. Ick." );

        // If we're not already the last entry...
        const index_type lastCellIndex = m_cells.size() - 1;
        if ( oldCellIndex != lastCellIndex )
        {
            // Put last cell into this position.
            // CJH HACK (scary) - we're transferring the overflow pointers
            // by value here.
            oldCell = m_cells[lastCellIndex];

            // We need to make sure the unordered map has
            // this new assignment.
            m_unorderedMap[ oldCell.coord() ] = oldCellIndex;
        }

        // Decrease the number of spatial cells by 1.
        m_cells.resize( lastCellIndex );
    }
}

//-*****************************************************************************
// Add new element indices.
template <typename BLOCK, typename MBV>
void BaseCompactHashMap<BLOCK,MBV>
::addElement( index_type i_elemIndex,
              const cell_coord_type& i_newCellCoord )
{
    // Now add the particle to the new cell.
    // Find new cell, or make it.
    Cell* newCell = find( i_newCellCoord );
    if ( newCell )
    {
        newCell->addIndex( i_elemIndex, m_overflowAlloc );
    }
    else
    {
        // Must create a new cell!
        // Top-level cells don't get allocated the same way as
        // overflow cells.
        const index_type newCellIndex = m_cells.size();
        m_cells.resize( newCellIndex + 1 );
        m_cells[newCellIndex].init( i_newCellCoord, i_elemIndex );

        // Don't forget to add the new cell to the map.
        m_unorderedMap[i_newCellCoord] = newCellIndex;
    }
}

//-*****************************************************************************
template <typename BLOCK, typename MBV>
void BaseCompactHashMap<BLOCK,MBV>
::debugPrintOccupancyRatio()
{
    index_type numCells = m_cells.size();
    index_type numNearOverFlow = 0;
    index_type numOverFlow = 0;
    for ( index_type i = 0; i < numCells; ++i )
    {
        if ( m_cells[i].size() > ( Cell::max_block_size-5 ) )
        {
            ++numNearOverFlow;
        }

        if ( m_cells[i].size() > ( Cell::max_block_size ) )
        {
            ++numOverFlow;
        }
    }
    std::cout << "Number of cells near overflow: "
              << numNearOverFlow << ", out of: " << numCells
              << ", which is: "
              << 100.0f * ( ( float )numNearOverFlow+1 ) / 
                          ( ( float )std::max( numCells, 1 ) )
              << " percent." << std::endl;

    std::cout << "Number of cells overflowed: "
              << numOverFlow << ", out of: " << numCells
              << ", which is: "
              << 100.0f * ( ( float )numOverFlow+1 ) / 
                          ( ( float )std::max( numCells, 1 ) )
              << " percent." << std::endl;

    std::cout << "Overflow max block size: " << int( Cell::max_block_size )
              << std::endl;
}

//-*****************************************************************************
template <typename BLOCK, typename MBV>
template <typename CELL_COORD_VEC>
void BaseCompactHashMap<BLOCK,MBV>
::debugValidateElementIndexRange( index_type i_beginIndex,
                                  index_type i_endIndex,
                                  const CELL_COORD_VEC& i_cellCoords )
{
    for ( index_type i = i_beginIndex; i < i_endIndex; ++i )
    {
        const cell_coord_type& cellCoord = i_cellCoords[i];
        Cell* cell = find( cellCoord );
        EMLD_ASSERT( cell != NULL,
                        "Should have a cell for cell coordinate: "
                        << cellCoord
                        << " at index: " << i );
        EMLD_ASSERT( cell->coord() == cellCoord,
                        "Cell coord should match index coord." );

        EMLD_ASSERT( cell->contains( i ),
                        "Couldn't find element index: " << i
                        << " in cell for cell coordinate: " << cellCoord );
    }
}

} // End namespace CompactHashMap
} // End namespace EmldCore

#endif