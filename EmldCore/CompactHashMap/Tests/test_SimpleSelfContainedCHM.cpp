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

//-*****************************************************************************
// Let's assume that you have a list of elements, like particles or triangles
// or tetrahedra, that you want to spatially sort into a set of uniformly-sized
// voxels. Your elements have a position associated with them, but are not
// in any particular order, and we don't want to actually reorder the elements
// themselves.
//
// You don't know in advance which voxels are occupied, or even how many voxels
// there are.
//
// We call the voxels "cells" in this implementation, and they have a uniform
// size in space.
//
// The self-contained class in this test file will statically create, from
// an array of particle positions and a cell size,
// an iterable array of just the occupied cells, and also allow you to
// iterate over all the stored particle indices in a radius around any point
// in space.
//
//-*****************************************************************************


#if __cplusplus > 199711L
#define CHM_USE_CXX11 1
#else
#undef CHM_USE_CXX11
#endif

#include <ImathMath.h>
#include <ImathVec.h>
#include <ImathMatrix.h>
#include <ImathBox.h>
#include <ImathQuat.h>
#include <ImathColor.h>
#include <ImathFun.h>
#include <ImathRandom.h>
#include <ImathBoxAlgo.h>

#if CHM_USE_CXX11

#include <unordered_map>
#include <memory>
#include <type_traits>

#define CHM_SHARED_PTR std::shared_ptr
#define CHM_UNIQUE_PTR std::unique_ptr
#define CHM_STATIC_CONSTEXPR static constexpr

#else

#include <boost/smart_ptr.hpp>
#include <boost/unordered_map.hpp>

#define CHM_SHARED_PTR boost::shared_ptr
#define CHM_UNIQUE_PTR boost::scoped_ptr
#define CHM_STATIC_CONSTEXPR static const

#endif

#include <iostream>
#include <vector>
#include <map>
#include <deque>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <utility>
#include <stdlib.h>

//-*****************************************************************************
// Thread Building Blocks
//#include <tbb/tbb.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_scan.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_sort.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>
#include <tbb/spin_rw_mutex.h>
#include <tbb/atomic.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/task.h>
//-*****************************************************************************

#include <sys/types.h>
#include <stdio.h>
#include <limits>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <assert.h>


namespace SimpleTestCHM {

//-*****************************************************************************
// CHM_VERBOSE_OUTPUT
#ifndef CHM_NO_VERBOSE

#define CHM_VERBOSE_OUT( TEXT )                                              \
do                                                                        \
{                                                                         \
    std::cout << TEXT << std::endl;                                       \
}                                                                         \
while( 0 )

#else

#define CHM_VERBOSE_OUT( TEXT ) do { /* nothing */; } while ( 0 )

#endif

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// TYPEDEFS
// Rather than templating all the classes on a base type, which can get a
// little verbose and tedious, instead we're here choosing the base positional
// vector type and coordinate vector type, as well as the base precision of
// each of those types.
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

typedef int32_t IntT;
typedef uint32_t UintT;
typedef float FloatT;

typedef int64_t IndexT;

//-*****************************************************************************
// Everything after this makes no precision assumptions.

typedef Imath::Vec3<IntT> V3IT;
typedef Imath::Vec3<UintT> V3UIT;
typedef Imath::Vec3<FloatT> V3T;

typedef Imath::Box<V3IT> Box3IT;
typedef Imath::Box<V3UIT> Box3UIT;
typedef Imath::Box<V3T> Box3T;

typedef std::vector<V3IT> V3ITVector;
typedef std::vector<V3T> V3TVector;
typedef std::vector<IntT> IntTVector;
typedef std::vector<IndexT> IndexTVector;

//-*****************************************************************************
// TBB STUFF.
typedef tbb::blocked_range<IndexT> IndexTRange;

// This actually has a pretty big effect on performance, but it's unclear on
// exactly how to best set this value.  Intel recommends starting with a
// value of 10,000, and then recursively subdividing it, running the code
// with the smaller value, and comparing performance until it starts to
// get slower instead of faster. However, you'd really need to do this 
// on a function-by-function basis, which is sorta crazytown.
static const IndexT k_GrainSize = 1024;

//-*****************************************************************************
// This simple struct is used for sorting the buckets.
#pragma pack( push, 1 )
struct CellCoordAndIndex
{
    CellCoordAndIndex() : cellCoord( 0, 0, 0 ), index( 0 ) {}
    CellCoordAndIndex( const V3IT& i_coord, IntT i_index )
        : cellCoord( i_coord ), index( i_index ) {}
    V3IT cellCoord;
    IntT index;
};
#pragma pack( pop )

typedef std::vector<CellCoordAndIndex> CellCoordAndIndexVector;

//-*****************************************************************************
// A comparator for CellAndIndices.
struct CellCoordAndIndexComp
{
    bool operator()( const CellCoordAndIndex& i_a, 
                     const CellCoordAndIndex& i_b ) const
    {
        if ( i_a.cellCoord.z < i_b.cellCoord.z ) { return true; }
        else if ( i_a.cellCoord.z > i_b.cellCoord.z ) { return false; }

        else if ( i_a.cellCoord.y < i_b.cellCoord.y ) { return true; }
        else if ( i_a.cellCoord.y > i_b.cellCoord.y ) { return false; }

        else if ( i_a.cellCoord.x < i_b.cellCoord.x ) { return true; }
        else if ( i_a.cellCoord.x > i_b.cellCoord.x ) { return false; }

        else { return i_a.index < i_b.index; }
    }
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// GLOBAL UTILITY FUNCTIONS
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

// Getting the data (or const data) pointers from vectors.

//-*****************************************************************************
// C++ 11 guarantees that vectors have a "data" member function.
// However, older C++ does not. Create a vector_data function to
// access the data.
#if CHM_USE_CXX11

template <typename VECTOR>
inline typename VECTOR::pointer vector_data( VECTOR& i_vec )
{
    return i_vec.data();
}

template <typename VECTOR>
inline typename VECTOR::const_pointer vector_cdata( const VECTOR& i_vec )
{
    return i_vec.data();
}

#else

template <typename VECTOR>
inline typename VECTOR::pointer vector_data( VECTOR& i_vec )
{
    if ( i_vec.size() == 0 )
    {
        return reinterpret_cast<typename VECTOR::pointer>( 0 );
    }
    else
    {
        return reinterpret_cast<typename VECTOR::pointer>( &( i_vec.front() ) );
    }
}

template <typename VECTOR>
inline typename VECTOR::const_pointer vector_cdata( const VECTOR& i_vec )
{
    if ( i_vec.size() == 0 )
    {
        return reinterpret_cast<typename VECTOR::const_pointer>( 0 );
    }
    else
    {
        return reinterpret_cast<typename VECTOR::const_pointer>(
                   &( i_vec.front() ) );
    }
}

#endif

//-*****************************************************************************
//-*****************************************************************************
// POSITION TO CELL COORDINATE
//-*****************************************************************************
//-*****************************************************************************
inline V3IT PosToCellCoord( const V3T& i_pos, const V3T& i_cellSize )
{
    return V3IT( ( IntT )Imath::Math<FloatT>::floor( i_pos.x / i_cellSize.x ),
                 ( IntT )Imath::Math<FloatT>::floor( i_pos.y / i_cellSize.y ),
                 ( IntT )Imath::Math<FloatT>::floor( i_pos.z / i_cellSize.z ) );
}

//-*****************************************************************************
//-*****************************************************************************
// CELL COORDINATE TO SPATIAL HASH
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// These large prime numbers used for hashing come from the paper,
// "Optimized Spatial Hashing for Collision Detection of Deformable Objects",
// by Matthias Tescher, Bruno Heidelberger, Matthias Muller,
// Danat Pomeranets, and Markus Gross
inline std::size_t SpatialHash( std::size_t i_x,
                                std::size_t i_y,
                                std::size_t i_z )
{
    return ( i_x * 73856093 ) ^
           ( i_y * 19349663 ) ^
           ( i_z * 83492791 );
}

//-*****************************************************************************
//inline std::size_t SpatialHash( const V3UIT& i_cell )
//{
//    return SpatialHash( i_cell.x, i_cell.y, i_cell.z );
//}

//-*****************************************************************************
inline std::size_t SpatialHash( const V3IT& i_cell )
{
    return SpatialHash( reinterpret_cast<const UintT&>( i_cell.x ),
                        reinterpret_cast<const UintT&>( i_cell.y ),
                        reinterpret_cast<const UintT&>( i_cell.z ) );
}

//-*****************************************************************************
struct SpatialHashFunctor
{
    std::size_t operator()( const V3IT& i_vec ) const
    {
        return SpatialHash( i_vec );
    }
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// THE SIMPLE COMPACT HASH MAP
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
class SimpleCompactHashMap
{
public:
    // The Simple CHM consists of a vector of sorted particle indices,
    // and a vector of cells. The cells contain a begin and end index
    // into that sorted index vector, and the two together represent the
    // storage of particle indices into cells, and the creation of cells.
    // This is the simple cell struct.
#pragma pack( push, 1 )
    struct Cell
    {
        Cell() : coord( 0, 0, 0 ), begin( 0 ), end( 0 ) {}
        Cell( const V3IT& i_cellCoord, IndexT i_b, IndexT i_e ) 
            : coord( i_cellCoord ), begin( i_b ), end( i_e ) {}
        
        V3T coord;
        IndexT begin;
        IndexT end;

        std::size_t size() const { return ( std::size_t )( end - begin ); }

        //-*********************************************************************
        // Use this iterator to iterate over the particle indices that belong
        // to a particular cell, as sorted by a particle compact hash map.
        template <typename CELL>
        class TParticleIndexIterator
        {
        public:
            // Empty constructor
            TParticleIndexIterator()
                : m_cell()
                , m_sortedIndices( NULL )
                , m_iter( 0 )
            {}

            // The constructor takes a cell and a compact hash map.
            TParticleIndexIterator( const CELL& i_cell, 
                                   const SimpleCompactHashMap& i_chm )
                : m_cell( i_cell )
                , m_sortedIndices( &( i_chm.sortedIndices() ) )
                , m_iter( m_cell.begin )
            {
                //std::cout << "CellPiter. coord: " << m_cell.coord << std::endl
                //          << "begin: " << m_cell.begin << std::endl
                //          << "end: " << m_cell.end << std::endl
                //          << "iter: " << m_iter << std::endl;
                    
            }

            // Is the iterator done?
            bool done() const
            {
                //std::cout << "Am I done? " << std::endl
                //          << "begin: " << m_cell.begin << std::endl
                //          << "end: " << m_cell.end << std::endl
                //          << "iter: " << m_iter << std::endl;

                return ( m_iter >= m_cell.end );
            }

            // Is the iterator not done?
            operator bool() const
            {
                return !done();
            }

            // Is the iterator done (!valid)?
            bool operator!() const
            {
                return done();
            }

            // Return the cell that this iterator is iterating over.
            const CELL& cell() const { return m_cell; }

            // Return the vector of sorted particle indices used by the
            // iterator.
            const IndexTVector& sortedIndices() const 
            { return *m_sortedIndices; }
            
            // ++iter (prefix)
            TParticleIndexIterator& operator++()
            {
                if ( m_iter < m_cell.end ) { ++m_iter; }
                return *this;
            }

            // iter++ (postfix)
            TParticleIndexIterator operator++( int )
            {
                TParticleIndexIterator ret( *this );
                ++( *this );
                return ret;
            }

            // Return the particle index. Error if iterator is invalid.
            IndexT operator*() const { return (*m_sortedIndices)[m_iter]; }

            // Are we pointed to the beginning of the cell?
            bool atBegin() const { return m_iter == m_cell.begin; }

            // Reset.
            void reset() { m_iter = m_cell.begin; }

        protected:
            CELL m_cell;
            const IndexTVector* m_sortedIndices;
            IndexT m_iter;
        }; // End of cell particle-index-iterator.

        typedef TParticleIndexIterator<Cell> ParticleIndexIterator;
    };
#pragma pack( pop )

    // Always typedef your vectors!
    typedef std::vector<Cell> CellVector;

    // Provide the constructor with a list of positions representing your
    // elements. The hash map will build only the cells that are occupied
    // by these particles, where the cell address is computed by the function
    // above.
    SimpleCompactHashMap( const V3TVector& i_positions,
                          const V3T& i_cellSize );

    // Given a Cell Coordinate, find the occupied cell that is at that
    // coordinate, or NULL if that coordinate is not occupied.
    const Cell* findCell( const V3IT& i_cellCoord ) const
    {
        InternalUnorderedMap::const_iterator fiter =
            m_cellIndexHashMap.find( i_cellCoord );
        if ( fiter == m_cellIndexHashMap.end() )
        {
            return NULL;
        }
        else
        {
            return &( m_cells[( *fiter ).second ] );
        }
    }

    // Is a cell coordinate occupied?
    bool isCellOccupied( const V3IT& i_cellCoord ) const
    {
        return ( findCell( i_cellCoord ) != NULL );
    }

    // How many cells?
    std::size_t size() const { return m_cells.size(); }
    
    // Direct access to internals - better to use iterators, though.
    const V3T& cellSize() const { return m_cellSize; }
    const IndexTVector& sortedIndices() const { return m_sortedIndices; }
    const CellVector& cells() const { return m_cells; }

    //-*************************************************************************
    // ITERATORS OVER THE WHOLE MAP
    //-*************************************************************************

    //-*************************************************************************    
    // Iterate over all the cells, and all the particles in those cells.
    //-*************************************************************************
    class AllParticleIndexIterator
    {
    public:

        // The constructor takes a compact hash map.
        explicit AllParticleIndexIterator( const SimpleCompactHashMap& i_chm )
            : m_begin( i_chm.cells().begin() )
            , m_end( i_chm.cells().end() )
            , m_iter( i_chm.cells().begin() )
            , m_chm( i_chm )
        {
            if ( m_iter != m_end )
            {
                m_cellIter = Cell::ParticleIndexIterator( *m_iter, m_chm );
            }
        }

        // Is the iterator done?
        bool done() const
        {
            return !( m_iter != m_end );
        }

        // Is the iterator not done?
        operator bool() const
        {
            return !done();
        }

        // Is the iterator done (!valid)?
        bool operator!() const
        {
            return done();
        }

        // Return the cell that this iterator is iterating over.
        // Only valid if the iterator is valid.
        const Cell& cell() const { return m_cellIter.cell(); }
        
        // ++iter (prefix)
        AllParticleIndexIterator& operator++()
        {
            if ( m_iter != m_end )
            {
                ++m_cellIter;
                if ( m_cellIter.done() )
                {
                    ++m_iter;
                    if ( m_iter != m_end )
                    {
                        m_cellIter = 
                            Cell::ParticleIndexIterator( *m_iter, m_chm );
                    }
                }
            }
            return *this;
        }

        // iter++ (postfix)
        AllParticleIndexIterator operator++( int )
        {
            AllParticleIndexIterator ret( *this );
            ++( *this );
            return ret;
        }

        // Return the particle index. Error if iterator is invalid.
        IndexT operator*() const { return *m_cellIter; }

        // Are we pointed to the beginning of the cell?
        bool atCellBegin() const { return ( !done() && m_cellIter.atBegin() ); }

        // Reset.
        void reset()
        { 
            m_iter = m_begin; 
            if ( m_iter != m_end )
            {
                m_cellIter = Cell::ParticleIndexIterator( *m_iter, m_chm );
            }
        }

    protected:
        CellVector::const_iterator m_begin;
        CellVector::const_iterator m_end;
        CellVector::const_iterator m_iter;
        const SimpleCompactHashMap& m_chm;

        Cell::ParticleIndexIterator m_cellIter;
    };

    //-*************************************************************************
    // Iterate over all the indices of all the particles within one full
    // cell size distance of the particle. (so, the 27-cell neighborhood with
    // the cell coordinate at the center).
    //-*************************************************************************
    class NbhdParticleIndexIterator
    {
    private:
        // This function assumes coord is inside coordBounds.
        void initCellIterFromCoord()
        {
            assert( m_coordBounds.intersects( m_coord ) );

            // Find a cell in the map at this coordinate.
            const Cell* cell = m_chm.findCell( m_coord );
            if ( cell )
            {
                // Found a cell - set up its iterator.
                m_cellIter = Cell::ParticleIndexIterator( *cell, 
                                                          m_chm );
            }
            else
            {
                // The empty iterator always returns true for done.   
                m_cellIter = Cell::ParticleIndexIterator();
            }
        }

        // Finds the next existing cell in the neighborhood, or bails.
        void ifCellIterDoneIterateUntilNextValidCoordOrAllDone()
        {
            // If the cell iterator is done, keep moving through the
            // bounds until we find a cell where we can make a new valid
            // cell iterator, or exit the bounds.
            while ( m_cellIter.done() )
            {
                ++m_coord.x;
                if ( m_coord.x > m_coordBounds.max.x )
                {
                    m_coord.x = m_coordBounds.min.x;
                    ++m_coord.y;
                    if ( m_coord.y > m_coordBounds.max.y )
                    {
                        m_coord.y = m_coordBounds.min.y;
                        ++m_coord.z;
                    }
                }
                
                // If we've exited the bounds, break out of the
                // while loop.
                if ( !m_coordBounds.intersects( m_coord ) ) { break; }

                // Init the cell iterator from the coord.
                initCellIterFromCoord();
            }
        }

    public:
        // Iterate over the 27 cells surrounding a coordinate
        NbhdParticleIndexIterator( const V3IT& i_centerCoord,
                                   const SimpleCompactHashMap& i_chm )
            : m_chm( i_chm )
        {
            m_coordBounds.min = i_centerCoord - V3IT( 1 );
            m_coordBounds.max = i_centerCoord + V3IT( 1 );
            m_coord = m_coordBounds.min;
            initCellIterFromCoord();
            ifCellIterDoneIterateUntilNextValidCoordOrAllDone();
        }

        // Iterate over all the cells in some bounds.
        NbhdParticleIndexIterator( const Box3IT& i_coordBounds,
                                   const SimpleCompactHashMap& i_chm )
            : m_coordBounds( i_coordBounds )
            , m_chm( i_chm )
        {
            m_coord = m_coordBounds.min;
            initCellIterFromCoord();
            ifCellIterDoneIterateUntilNextValidCoordOrAllDone();
        }

        // Is the iterator done?
        bool done() const
        {
            return m_cellIter.done();
        }

        // Is the iterator not done?
        operator bool() const
        {
            return !done();
        }

        // Is the iterator done (!valid)?
        bool operator!() const
        {
            return done();
        }

        // Return the cell that this iterator is iterating over.
        // Only valid if the iterator is valid.
        const Cell& cell() const { return m_cellIter.cell(); }

        // ++iter
        NbhdParticleIndexIterator& operator++()
        {
            // Only bother iterating if we're a valid iterator.
            if ( m_cellIter )
            {
                // Increment the cell iterator
                ++m_cellIter;

                // And iterate until next valid coord or done.
                // This won't move if the cell iterator is not done.
                ifCellIterDoneIterateUntilNextValidCoordOrAllDone();                
            }
        }

        // iter++ (postfix)
        NbhdParticleIndexIterator operator++( int )
        {
            NbhdParticleIndexIterator ret( *this );
            ++( *this );
            return ret;
        }

        // Return the particle index. Error if iterator is invalid.
        IndexT operator*() const { return *m_cellIter; }

        // Are we pointed to the beginning of the cell?
        bool atCellBegin() const { return ( !done() && m_cellIter.atBegin() ); }

        // Reset.
        void reset()
        { 
            m_coord = m_coordBounds.min;
            initCellIterFromCoord();
            ifCellIterDoneIterateUntilNextValidCoordOrAllDone();
        }

    protected:
        V3IT m_coord;
        Box3IT m_coordBounds;
        const SimpleCompactHashMap& m_chm;

        Cell::ParticleIndexIterator m_cellIter;
    };

protected:
    //-*************************************************************************
    // UNORDERED MAP TYPEDEF
    //-*************************************************************************
#if CHM_USE_CXX11
    typedef std::unordered_map<V3IT, IndexT, 
                               SpatialHashFunctor,  
                               std::equal_to<V3IT> >
    InternalUnorderedMap;
#else
    typedef boost::unordered_map<V3IT, IndexT, 
                                 SpatialHashFunctor,  
                                 std::equal_to<V3IT> >
    InternalUnorderedMap;
#endif

    // The cell size.
    V3T m_cellSize;

    // The sorted particle indices.
    IndexTVector m_sortedIndices;

    // The cells.
    CellVector m_cells;

    // The internal unordered map
    InternalUnorderedMap m_cellIndexHashMap;
};

//-*****************************************************************************
//-*****************************************************************************
// PARALLEL FUNCTORS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// Creates the CellAndIndices from the Positions
struct InitCellCoordAndIndicesFromPositions
{
    const V3T* Positions;
    CellCoordAndIndex* CellCoordAndIndices;
    V3T CellSize;

    void operator()( const IndexTRange& i_range ) const
    {
        for ( IndexT i = i_range.begin(); i != i_range.end(); ++i )
        {
            CellCoordAndIndices[i] =
                CellCoordAndIndex( PosToCellCoord( Positions[i], CellSize ), 
                                    i );
        }
    }
};

//-*****************************************************************************
// Copies the sorted indices out of the CellCoordAndIndex struct into a regular
// array.
struct CopySortedIndices
{
    const CellCoordAndIndex* CellCoordAndIndices;
    IndexT* SortedIndices;
    
    void operator()( const IndexTRange& i_range ) const
    {
        for ( IndexT i = i_range.begin(); i != i_range.end(); ++i )
        {
            SortedIndices[i] = CellCoordAndIndices[i].index;
        }
    }
};

//-*****************************************************************************
// This is complex - this is the functor that tbb needs in order to compute
// a prefix sum at every location in an array, in two passes.
// The easiest way to understand this is read through the tbb::parallel_scan
// documentation, looking specifically for parallel prefix sum.
//-*****************************************************************************
struct ContiguousCellPrefixSum
{
    const CellCoordAndIndex* CellCoordAndIndices;
    IndexT* CellIndices;
    IndexT Sum;

    ContiguousCellPrefixSum( const CellCoordAndIndex* i_cellCoordAndIndices,
                             IndexT* o_cellIndices )
        : CellCoordAndIndices( i_cellCoordAndIndices )
        , CellIndices( o_cellIndices )
        , Sum( 0 )
    {}

    ContiguousCellPrefixSum( ContiguousCellPrefixSum& i_copy, tbb::split )
        : CellCoordAndIndices( i_copy.CellCoordAndIndices )
        , CellIndices( i_copy.CellIndices )
        , Sum( 0 )
    {}

    // Called by parallel scan.
    template <typename TAG>
    void operator()( const IndexTRange& i_range, TAG i_tag )
    {
        IndexT temp = Sum;
        for ( IndexT i = i_range.begin(); i != i_range.end(); ++i )
        {
            // We're adding one to "temp" if this is the start of a new
            // cell that is not the 0'th cell.
            if ( i != 0 &&
                ( CellCoordAndIndices[i-1].cellCoord != 
                  CellCoordAndIndices[i].cellCoord ) )
            {
                ++temp;
            }

            // parallel-scan is a multi-stage process. If it's the last
            // stage, we copy the values into our output array.
            if ( TAG::is_final_scan() )
            {
                CellIndices[i] = temp;
            }
        }

        // Re-set the sum.
        Sum = temp;
    }

    // This joins two segments, in preparation for the final scan.
    void reverse_join( ContiguousCellPrefixSum& i_other )
    {
        Sum += i_other.Sum;
    }

    // This copies one segment to the other.
    void assign( ContiguousCellPrefixSum& i_other )
    {
        Sum = i_other.Sum;
    }
};

//-*****************************************************************************
// This functor sets the cell begin and end calipers, as well as setting
// the cell coordinates.
struct InitCells
{
    SimpleCompactHashMap::Cell* Cells;
    const CellCoordAndIndex* CellCoordAndIndices;
    const IndexT* CellIndices;
    std::size_t N;

    void operator()( const IndexTRange& i_range ) const
    {
        for ( IndexT i = i_range.begin(); i != i_range.end(); ++i )
        {
            const IndexT thisCellIndex = CellIndices[i];
            SimpleCompactHashMap::Cell& cell = Cells[thisCellIndex];

            if ( i == 0 ) 
            {
                // If we're the 0th point, set the 0th cell begin and cell.
                cell.coord = CellCoordAndIndices[i].cellCoord;
                cell.begin = 0;
            }
            else
            {
                const IndexT prevCellIndex = CellIndices[i-1];

                // Set the beginning of this cell, and the end of the previous
                // cell, if we're different than the previous point.
                if ( prevCellIndex != thisCellIndex )
                {
                    SimpleCompactHashMap::Cell& prevCell = Cells[prevCellIndex];
                    prevCell.end = i;
                    cell.coord = CellCoordAndIndices[i].cellCoord;
                    cell.begin = i;
                }
            }

            // If we're the last cell, set the end.
            if ( i == N-1 )
            {
                cell.end = i;
            }
        }
    }
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// IMPLEMENTATION OF SIMPLE COMPACT HASH MAP IN TERMS OF FUNCTORS ABOVE
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
SimpleCompactHashMap::SimpleCompactHashMap( const V3TVector& i_positions,
                                            const V3T& i_cellSize )
    : m_cellSize( i_cellSize )
{
    // Get the size of the particles array.
    std::size_t N = i_positions.size();

    //-*************************************************************************
    // First create the cells and indices.
    // This step takes the particle positions and produces a list of
    // ordered pairs of index and cell coordinate that would look like:
    // {{-7,19,32},0}
    // {{-8,19,31},1}
    // ...
    // {{15,6,30},N-1}
    CellCoordAndIndexVector cellCoordAndIndices( N );
    // Init them in parallel.
    {
        InitCellCoordAndIndicesFromPositions F;
        F.Positions = vector_cdata( i_positions );
        F.CellCoordAndIndices = vector_data( cellCoordAndIndices );
        F.CellSize = m_cellSize;
        tbb::parallel_for( IndexTRange( 0, N, k_GrainSize ), F );
    }
    CHM_VERBOSE_OUT( "Created " << N << " CellAndIndices" );

    //for ( std::size_t i = 0; i < N; ++i )
    //{
    //    std::cout << "CellCoord[" << i << "] = " 
    //              << cellCoordAndIndices[i].cellCoord << ", "
    //              << "i = " << cellCoordAndIndices[i].index << std::endl;
    //}

    //-*************************************************************************
    // Now sort the cells and indices.  We have a comparator for CellCoordAndIndex
    // vectors, so it's pretty straightforward.
    // This step will group all of the points that lie in the same cell together
    // and then they'll be sorted by index within each cell group, like this:
    // {{15,6,30},734}
    // {{15,6,30},882}
    // {{15,6,30},993}
    // {{15,6,30},N-1}
    // ...
    // {{-7,19,32},0}
    // {{-7,19,32},38}
    // {{-7,19,32},41}
    tbb::parallel_sort( cellCoordAndIndices.begin(), cellCoordAndIndices.end(),
                        CellCoordAndIndexComp() );
    CHM_VERBOSE_OUT( "Sorted " << N << " CellAndIndices" );

    //for ( std::size_t i = 0; i < N; ++i )
    //{
    //    std::cout << "CellCoord[" << i << "] = " 
    //              << cellCoordAndIndices[i].cellCoord << ", "
    //              << "i = " << cellCoordAndIndices[i].index << std::endl;
    //}

    //-*************************************************************************
    // Now copy cell indices into our sorted indices array. This just keeps
    // the sorted indices from the above step, so we have something like:
    // {734,882,993,N-1, ..., 0,38,41}
    m_sortedIndices.resize( N );
    {
        CopySortedIndices F;
        F.CellCoordAndIndices = vector_cdata( cellCoordAndIndices );
        F.SortedIndices = vector_data( m_sortedIndices );
        tbb::parallel_for( IndexTRange( 0, N, k_GrainSize ), F );

#if 0
        // The lambda version:
        const CellCoordAndIndex* CellCoordAndIndices = 
            vector_cdata( cellCoordAndIndices );
        IndexT* SortedIndices = vector_data( m_sortedIndices );
        tbb::parallel_for( IndexTRange( 0, N, k_GrainSize ),
            [=](const tbb::blocked_range<IndexT>& i_range )
            {
                for( IndexT i=i_range.begin(); i!=i_range.end(); ++i )
                {  
                    SortedIndices[i] = CellCoordAndIndices[i].index;
                }
            } 
            ); // End of parallel for
#endif

    }
    CHM_VERBOSE_OUT( "Copied " << N << " Sorted Indices" );

    //-*************************************************************************
    // Now the somewhat complex part. We define a function wrapper around
    // the CellsAndIndices which, at an index i, returns 1 if the cell coord
    // at that index is different from the cell coord at i-1, (or 0 for i=0)
    // So this array, which we would never create directly, looks like this for
    // our above example.
    // {0,0,0,0,1,...1,0,0}
    // So the 1's basically indicate the beginning of new contiguous cells
    // in the data, except at the 0'th position.
    // We then perform a parallel prefix sum on that virtual array, which
    // produces an array that looks like this:
    // {0,0,0,0,1,1,1,2,2,2,2,.....,M-1,M-1,M-1}
    // Where M is the number of unique cells.
    // The produced array then acts as the index of the cell for each sorted
    // particle, in an as-yet-uncreated array of cells.
    IndexTVector cellIndices( N );
    {
        ContiguousCellPrefixSum F( vector_cdata( cellCoordAndIndices ),
                                   vector_data( cellIndices ) );
        tbb::parallel_scan( IndexTRange( 0, N, k_GrainSize ), F );
    }
    CHM_VERBOSE_OUT( "Created " << N << " Contiguous Cell Prefix Sums" );
    
    //-*************************************************************************
    // We now know how many cells to create.
    // The last address in the cellIndices (if there are any) is M-1, where
    // M is the number of cells.
    const IndexT numCells = ( N > 0 ) ? ( cellIndices.back() + 1 ) : 0;
    m_cells.resize( numCells );
    CHM_VERBOSE_OUT( "Created " << numCells << " Cells" );

    //-*************************************************************************
    // Now we have to fill those cells with the begin and index of the 
    // sorted particle indices. This is a loop over the particles, but for
    // most of them, there's no actual work to do.
    {
        InitCells F;
        F.Cells = vector_data( m_cells );
        F.CellCoordAndIndices = vector_cdata( cellCoordAndIndices );
        F.CellIndices = vector_cdata( cellIndices );
        F.N = N;
        tbb::parallel_for( IndexTRange( 0, N, k_GrainSize ), F );
    }
    CHM_VERBOSE_OUT( "Initialized " << numCells << " Cells" );

    //-*************************************************************************
    // Finally, put the cell coordinates into a hash map. This is a serial
    // operation.

    // Set the number of hash buckets.
    m_cellIndexHashMap.rehash( numCells * 2 );

    // Insert the cells.
    for ( IndexT i = 0; i < numCells; ++i )
    {
        const Cell& cell = m_cells[i];

        // Some hard-core validity testing.
#ifdef DEBUG
        for ( IndexT j = cell.begin; j != cell.end; ++j )
        {
            // Verify that the CellCoordAndIndex pointed to by this sorted
            // index is the right one.
            assert( cellCoordAndIndices[j].cellCoord == cell.coord );

            // Go all the way back to the unsorted particles to see whether
            // the converted position is the correct one.
            assert( PosToCellCoord( i_positions[m_sortedIndices[j]], 
                    m_cellSize ) 
                    == cell.coord );
        }
#endif

        m_cellIndexHashMap[ cell.coord ] = i;
    }
    CHM_VERBOSE_OUT( "Populated unordered map with " << numCells 
                 << " Cell Indices" );
}

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// TEST THIS OUT!!!
// What's a good way to test this thing? Generate random points inside
// a volume.
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// This will init the particle positions randomly, but the same time every time.
struct InitRandPositions
{
    V3T* Positions;
    Box3T Bounds;
    
    void operator()( const IndexTRange& i_range ) const
    {
        Imath::Rand48 randGen;
        for ( IndexT i = i_range.begin(); i != i_range.end(); ++i )
        {
            randGen.init( i );
            Positions[i] = V3T( randGen.nextf( Bounds.min.x, Bounds.max.x ),
                                randGen.nextf( Bounds.min.y, Bounds.max.y ),
                                randGen.nextf( Bounds.min.z, Bounds.max.z ) );
        }                                                  
    }
};

//-*****************************************************************************
// This will build a system using the functor above.
class TestParticles
{
public:
    TestParticles( FloatT i_worldSize, FloatT i_cellSize,
                   std::size_t i_numParts )
        : m_bounds( V3T( -i_worldSize/2.0f ), V3T( i_worldSize/2.0f ) )
        , m_cellSize( i_cellSize )
        , m_positions( i_numParts )
    {
        InitRandPositions F;
        F.Positions = vector_data( m_positions );
        F.Bounds = m_bounds;
        tbb::parallel_for( IndexTRange( 0, i_numParts, k_GrainSize ), F );
        CHM_VERBOSE_OUT( "Created: " << i_numParts << " random points." );

        // Build the compact hash map.
        m_compactHashMap.reset( new SimpleCompactHashMap( m_positions, 
                                                          m_cellSize ) );
        CHM_VERBOSE_OUT( "Created SimpleCHM." );
    }

    // This test will iterate over all the points & cells.
    void test1( std::size_t i_cellPrintFreq = 1000,
                std::size_t i_partPrintFreq = 10000 )
    {
        CHM_VERBOSE_OUT( "Test1 Begin" );
        bool doCellPrint = false;
        
        SimpleCompactHashMap::AllParticleIndexIterator 
            iter( *m_compactHashMap );
        for ( ; iter; ++iter )
        {
            if ( iter.atCellBegin() )
            {
                std::size_t cellCoordHash = SpatialHash( iter.cell().coord );
                if ( ( cellCoordHash%i_cellPrintFreq ) == 0 )
                {
                    doCellPrint = true;
                    const SimpleCompactHashMap::Cell& cell = iter.cell();
                        CHM_VERBOSE_OUT( "Cell at coord: " << cell.coord 
                                     << " begin, with " << cell.size()
                                    << " points in it" );
                }
                else
                {
                    doCellPrint = true;
                }
            }
            if ( doCellPrint )
            {
                IndexT partIndex = (*iter);
                if ( ( partIndex % i_partPrintFreq ) == 0 )
                {
                    CHM_VERBOSE_OUT( "particle index: " << (*iter) );
                }
            }
        }

        CHM_VERBOSE_OUT( "Test1 End" );
    }

    // This test will iterate over all the points in a neighborhood around
    // a certain point.
    void test2()
    {
        CHM_VERBOSE_OUT( "Test2 Begin" );
        
        SimpleCompactHashMap::NbhdParticleIndexIterator 
            iter( V3IT( 0, 0, 0 ), *m_compactHashMap );
        for ( ; iter; ++iter )
        {
            if ( iter.atCellBegin() )
            {
                const SimpleCompactHashMap::Cell& cell = iter.cell();
                CHM_VERBOSE_OUT( "Cell at coord: " << cell.coord 
                             << " begin, with " << cell.size()
                             << " points in it" );
            }
            CHM_VERBOSE_OUT( "particle index: " << (*iter) );
        }

        CHM_VERBOSE_OUT( "Test2 End" );
    }

    const V3TVector& positions() const { return m_positions; }

protected:
    Box3T m_bounds;
    V3T m_cellSize;
    V3TVector m_positions;
    CHM_UNIQUE_PTR<SimpleCompactHashMap> m_compactHashMap;
};

} // End namespace SimpleTestCHM

struct DummyVec3
{
    float a;
    float b;
    float c;
};

//-*****************************************************************************
int main( int argc, char *argv[] )
{
    SimpleTestCHM::IntT numParts = 100000000;
    SimpleTestCHM::FloatT worldSize = 100.0f;
    SimpleTestCHM::FloatT cellSize = 2.5f;
    SimpleTestCHM::TestParticles testp( worldSize, cellSize, numParts );
    std::size_t cellPrintFreq = 10000;
    std::size_t partPrintFreq = 10000;
    
    //testp.test1( cellPrintFreq, partPrintFreq );
    
    //testp.test2();

    const std::vector<DummyVec3>& imathVecs =
        reinterpret_cast<const std::vector<DummyVec3>&>( testp.positions() );

    return 0;
}
