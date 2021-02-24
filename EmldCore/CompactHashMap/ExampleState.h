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

#ifndef _EmldCore_CompactHashMap_ExampleState_h_
#define _EmldCore_CompactHashMap_ExampleState_h_

#include "Foundation.h"
#include "StateReorder.h"
#include "VectorManager.h"
#include "IndexBlock.h"
#include "StateCompactHashMap.h"

namespace EmldCore {
namespace CompactHashMap {
namespace Example {

//-*****************************************************************************
// Emit information for state.
#pragma pack( push, 1 )
struct EmitInfo
{
    V3f pos;
    V3f vel;
    float mass;
    float age;
    float lifespan;

    EmitInfo() : pos( 0.0f ), vel( 0.0f ), mass( 0.0f )
    , age( 0.0f ), lifespan( 0.0f ) {}
    EmitInfo( const V3f& i_p, const V3f& i_v, float i_m,
    float i_a, float i_ls )
        : pos( i_p ), vel( i_v ), mass( i_m )
        , age( i_a ), lifespan( i_ls ) {}

    struct less_than_comp
    {
        bool operator()( const EmitInfo& i_a, const EmitInfo& i_b ) const
        {
            if ( i_a.age < i_b.age ) { return true; }
            else if ( i_a.age > i_b.age ) { return false; }
            else if ( i_a.pos < i_b.pos ) { return true; }
            else if ( i_a.pos > i_b.pos ) { return false; }
            else if ( i_a.mass < i_b.mass ) { return true; }
            else if ( i_a.mass > i_b.mass ) { return false; }
            else if ( i_a.lifespan < i_b.lifespan ) { return true; }
            else if ( i_a.lifespan > i_b.lifespan ) { return false; }
            else { return ( i_a.vel < i_b.vel ); }
        }
    };
};
#pragma pack( pop )

//-*****************************************************************************
class VectorManager
    : public VectorManagerWrapper<VectorManager>
    // ints and floats are 4 bytes.
    , public SubVectorManager<storage_bytes_4>
    // v3i and v3f are 12 bytes. (3*4)
    , public SubVectorManager<storage_bytes_12>
    // sort-tuples are (3*4) + (3*4) + 4 + 4 + 4 = 9*4 = 36
    // emit infos are (3*4) + (3*4) + 4 + 4 + 4 = 36
    , public SubVectorManager<storage_bytes_36>
    // cells are some number of indices, 128-bytes worth.
    , public SubVectorManager<storage_bytes_128>
{
public:
    EMLD_STATLIT_CONSTANT std::size_t kDefaultBucketSize = 4096;

    VectorManager()
        : VectorManagerWrapper<VectorManager> ()
        , SubVectorManager<storage_bytes_4>( kDefaultBucketSize )
        , SubVectorManager<storage_bytes_12>( kDefaultBucketSize )
        , SubVectorManager<storage_bytes_36>( kDefaultBucketSize )
        , SubVectorManager<storage_bytes_128>( kDefaultBucketSize )
    {}

    explicit VectorManager( std::size_t i_bucketSize )
        : VectorManagerWrapper<VectorManager> ()
        , SubVectorManager<storage_bytes_4>( i_bucketSize )
        , SubVectorManager<storage_bytes_12>( i_bucketSize )
        , SubVectorManager<storage_bytes_36>( i_bucketSize )
        , SubVectorManager<storage_bytes_128>( i_bucketSize )
    {}

    void setBucketSize( std::size_t i_bucketSize )
    {
        SubVectorManager<storage_bytes_4>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_12>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_36>::setBucketSize( i_bucketSize );
        SubVectorManager<storage_bytes_128>::setBucketSize( i_bucketSize );
    }
};

//-*****************************************************************************
// cell type.
typedef IndexBlock<V3i, int32_t, 128> cell_t;

//-*****************************************************************************
typedef Imath::Vec3<uint32_t> V3ui;

typedef VectorManager::TypedManagedCvectorHandle<int32_t> int_mvh;
typedef VectorManager::TypedManagedCvectorHandle<uint32_t> uint_mvh;
typedef VectorManager::TypedManagedCvectorHandle<float32_t> float_mvh;
typedef VectorManager::TypedManagedCvectorHandle<V3i> V3i_mvh;
typedef VectorManager::TypedManagedCvectorHandle<V3ui> V3ui_mvh;
typedef VectorManager::TypedManagedCvectorHandle<V3f> V3f_mvh;

// Make it 36 bytes so it's the same size as emit info.
typedef ReorderTuple<V3f, V3i, int32_t, 36> sort_tuple_t;

} // End namespace Example
} // End namespace CompactHashMap

namespace Util {

template <>
struct storage_traits<CompactHashMap::Example::EmitInfo>
{
    typedef CompactHashMap::Example::EmitInfo value_type;
    typedef storage_bytes_36 storage_type;
    typedef int32_t pod_type;
};

template <>
struct storage_traits<CompactHashMap::Example::sort_tuple_t>
{
    typedef CompactHashMap::Example::sort_tuple_t value_type;
    typedef storage_bytes_36 storage_type;
    typedef int32_t pod_type;
};

template <>
struct storage_traits<CompactHashMap::Example::cell_t>
{
    typedef CompactHashMap::Example::cell_t value_type;
    typedef storage_bytes_128 storage_type;
    typedef int32_t pod_type;
};

} // End namespace Util

namespace CompactHashMap {
namespace Example {

typedef VectorManager::TypedManagedCvectorHandle<EmitInfo> emit_mvh;
typedef VectorManager::TypedManagedCvectorHandle<sort_tuple_t> sort_tuple_mvh;
typedef VectorManager::TypedManagedCvectorHandle<cell_t> cell_mvh;

//-*****************************************************************************
class SimpleTransform
{
public:
    typedef V3f sim_coord_type;
    typedef V3i cell_coord_type;

    SimpleTransform( float i_cellSize ) : m_cellSize( i_cellSize ) {}

    float cellSize() const { return m_cellSize; }

    sim_coord_type cellToSim( const cell_coord_type& i_cell ) const
    {
        // The cell ( 0, 0, 0 ) goes from ( 0.0f, 0.0f, 0.0f ) to
        // ( cellWidth, cellWidth, cellWidth ) in space. We turn the cell
        // coordinate into the sim coordinate at the center of the cell.
        return sim_coord_type( 0.5f * m_cellSize ) +
               sim_coord_type( m_cellSize * float( i_cell.x ),
                               m_cellSize * float( i_cell.y ),
                               m_cellSize * float( i_cell.z ) );
    }

    cell_coord_type simToCell( const sim_coord_type& i_sim ) const
    {
        // We just return the floor of the coord divided by cell size.
        return cell_coord_type( ( int )floorf( i_sim.x / m_cellSize ),
                                ( int )floorf( i_sim.y / m_cellSize ),
                                ( int )floorf( i_sim.z / m_cellSize ) );
    }

protected:
    float m_cellSize;
};

//-*****************************************************************************
class TestState
{
public:
    typedef int32_t                                 index_type;

    typedef SimpleTransform                         transform_type;

    typedef V3f                                     sim_coord_type;
    typedef V3i                                     cell_coord_type;

    typedef cell_t                                  cell_type;
    typedef sort_tuple_t                            sort_tuple_type;
    typedef EmitInfo                                emit_type;

    typedef VectorManager                           vector_manager_type;
    typedef V3f_mvh                                 sim_coord_mvh_type;
    typedef V3i_mvh                                 cell_coord_mvh_type;
    typedef int_mvh                                 index_mvh_type;
    typedef sort_tuple_mvh                          sort_tuple_mvh_type;
    typedef cell_mvh                                cell_mvh_type;
    typedef emit_mvh                                emit_mvh_type;


    TestState( VectorManager& i_vecMgr,
               const SimpleTransform& i_transform )
        : m_vectorManager( i_vecMgr )
        , m_transform( i_transform )
        , m_nextId( 1 )
    {
        Ids = m_vectorManager.get<int32_t>( 0 );
        Positions = m_vectorManager.get<V3f>( 0 );
        Cells = m_vectorManager.get<V3i>( 0 );
        Velocities = m_vectorManager.get<V3f>( 0 );
        Masses = m_vectorManager.get<float32_t>( 0 );
        Ages = m_vectorManager.get<float32_t>( 0 );
        Lifespans = m_vectorManager.get<float32_t>( 0 );
    }

    index_type size() const { return Positions.size(); }

    vector_manager_type& vectorManager() { return m_vectorManager; }
    const transform_type& transform() const { return m_transform; }

    sim_coord_mvh_type simCoords() { return Positions; }
    cell_coord_mvh_type cellCoords() { return Cells; }


    void resize( int i_newSize )
    {
        Ids.resize( i_newSize );
        Positions.resize( i_newSize );
        Cells.resize( i_newSize );
        Velocities.resize( i_newSize );
        Masses.resize( i_newSize );
        Ages.resize( i_newSize );
        Lifespans.resize( i_newSize );
    }

    void sort()
    {
        StateReorder<TestState> r( *this );
    }

    void reorderExtraData( const StateReorder<TestState>& i_reorder )
    {
        const int N = size();
        {
            int_mvh tmpi = m_vectorManager.get<int32_t>( N );
            i_reorder.reorder( Ids, tmpi );
        }

        {
            float_mvh tmpf = m_vectorManager.get<float32_t>( N );
            i_reorder.reorder( Masses, tmpf );
            i_reorder.reorder( Ages, tmpf );
            i_reorder.reorder( Lifespans, tmpf );
        }

        {
            V3f_mvh tmpv = m_vectorManager.get<V3f>( N );
            i_reorder.reorder( Velocities, tmpv );
        }
    }

    void copyFromIndex( index_type i_src, index_type i_dst )
    {
        Ids[i_dst] = Ids[i_src];
        Positions[i_dst] = Positions[i_src];
        Cells[i_dst] = Cells[i_src];
        Velocities[i_dst] = Velocities[i_src];
        Masses[i_dst] = Masses[i_src];
        Ages[i_dst] = Ages[i_src];
        Lifespans[i_dst] = Lifespans[i_src];
    }

    void emitIntoIndex( const emit_type& i_emit, index_type i_index )
    {
        Ids[i_index] = m_nextId;
        Positions[i_index] = i_emit.pos;
        Cells[i_index] = m_transform.simToCell( i_emit.pos );
        Velocities[i_index] = i_emit.vel;
        Masses[i_index] = std::max( i_emit.mass, 0.01f );
        Ages[i_index] = i_emit.age;
        Lifespans[i_index] = i_emit.lifespan;

        ++m_nextId;
        if ( m_nextId < 1 ) { m_nextId = 1; }
    }

    void emitUnsorted( emit_mvh_type i_emitted )
    {
        index_type N = i_emitted.size();
        int_mvh emittedIndices = vectorManager().get<index_type>( N );
        ParallelUtil::VectorSetOrderedIndices( emittedIndices );
        {
            typedef emit_type::less_than_comp emit_comp_type;
            typedef EmitIndexComp<emit_type, 
                    emit_comp_type, index_type> eic_type;
            emit_comp_type ec;
            eic_type eic( i_emitted.cdata(), ec );
            ParallelUtil::VectorSort( emittedIndices, eic );
        }

        emitSortedByIndex( i_emitted.cdata(), 
                           emittedIndices.begin(), 
                           emittedIndices.end() );
    }

    void emitSortedByIndex( const emit_type* i_emitted,
                            const index_type* i_emitIndexBegin,
                            const index_type* i_emitIndexEnd )
    {
        index_type numToEmit = i_emitIndexEnd - i_emitIndexBegin;
        if ( numToEmit <= 0 ) { return; }

        index_type oldSize = size();
        resize( oldSize + numToEmit );

        for ( index_type i = 0; i < numToEmit; ++i )
        {
            emitIntoIndex( i_emitted[ i_emitIndexBegin[ i ] ], oldSize + i );
        }
    }

protected:
    vector_manager_type& m_vectorManager;
    const transform_type& m_transform;
    int m_nextId;

public:
    int_mvh Ids;
    V3f_mvh Positions;
    V3i_mvh Cells;

    V3f_mvh Velocities;
    float_mvh Masses;

    float_mvh Ages;
    float_mvh Lifespans;
};

//-*****************************************************************************
typedef StateCompactHashMap<TestState> TestStateCHM;

//-*****************************************************************************
class SimpleTestSim
{
public:
    SimpleTestSim( int numParts );

    void unsortedTimeStep( float i_dt );
    void timeStep( float i_dt );

    VectorManager& vectorManager() { return *m_vectorManager; }
    TestState& state() { return *m_state; }
    const TestState& state() const { return *m_state; }
    const SimpleTransform& transform() const { return *m_transform; }


protected:

    // Uses the current time as a seed.
    void emit( int N, float i_dt, emit_mvh o_emitted );

    EMLD_UNIQUE_PTR<VectorManager> m_vectorManager;
    EMLD_UNIQUE_PTR<SimpleTransform> m_transform;
    EMLD_UNIQUE_PTR<TestState> m_state;

    float m_currentTime;
};

//-*****************************************************************************
class HashedTestSim : public SimpleTestSim
{
public:
    HashedTestSim( int i_numParts );

    void timeStep( float i_dt );

protected:
    float m_emitRate;
    
    void killAndEmit( float i_dt );

    EMLD_UNIQUE_PTR<TestStateCHM> m_stateHashMap;
};

} // End namespace Example
} // End namespace CompactHashMap
} // End namespace EmldCore

#endif

