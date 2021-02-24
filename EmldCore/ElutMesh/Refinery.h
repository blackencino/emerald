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

#ifndef _EmldCore_ElutMesh_Refinery_h_
#define _EmldCore_ElutMesh_Refinery_h_

#include "Foundation.h"
#include "MeshBuilder.h"
#include "DicingTools.h"
#include "ProcessCells.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
// Emit info.
template <typename T>
struct EmitInfo
{
    Box3i cellBounds;
    Imath::Interval<T> levelSetRange;
    EmitInfo()
    {
        cellBounds.makeEmpty();
        levelSetRange.makeEmpty();
    }

    EmitInfo( const Box3i& i_cbounds, const Imath::Interval<T>& i_lsr )
        : cellBounds( i_cbounds )
        , levelSetRange( i_lsr )
        {}
};

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
class Refinery
{
public:
    typedef NODE node_type;
    typedef DICER dicer_type;
    typedef MESH mesh_type;

    typedef Refinery<NODE,DICER,MESH> this_type;

    typedef typename NODE::value_type value_type;
    typedef value_type T;
    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;
    typedef Imath::Interval<T> IvlT;
    typedef std::vector<IvlT> IvlTArray;

    typedef EmitInfo<T> emit_info;
    typedef epu::BucketedConcurrentVector<emit_info> emit_info_bcv;

    Refinery( NODE& io_node,
              const DICER& i_dicer,
              MESH& o_mesh );

    // This will recompute the bounds, it is called by step and refine.
protected:
    void recomputeBounds();

public:
    // This will keep stepping until a mesh is created, and then it
    // is a no-op.
    void step();

    // This steps all the way until a mesh is created.
    void refine();

    // Access to the arrays
    const V3iArray& cellMins() const { return m_state.cellMins; }
    const V3iArray& cellMaxs() const { return m_state.cellMaxs; }
    const IvlTArray& levelSetRanges() const { return m_state.levelSetRange; }

    // Access to the state information.
    int subdivsRemaining() const { return m_state.subdivsRemaining; }
    bool createdMesh() const { return m_state.createdMesh; }

    // The current bounds, in object space, at this stage.
    B3T bounds() const { return m_bounds; }

protected:
    void stepRefineCells();
    void stepCreateMesh();

    // Inputs
    NODE& m_node;
    const DICER& m_dicer;

    // Outputs
    MESH& m_mesh;

    // Struct of arrays
    struct State
    {
        V3iArray cellMins;
        V3iArray cellMaxs;
        IvlTArray levelSetRange;

        bool createdMesh;

        int subdivsRemaining;

        State() : createdMesh( false ), subdivsRemaining( 1000 ) {}
    } m_state;

    // Emit Info BCV
    emit_info_bcv m_emitInfoBCV;

    // Bounds in object space. Updated at every step.
    B3T m_bounds;
};

//-*****************************************************************************
//-*****************************************************************************
// IMPLEMENTATION
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename REFINE>
struct RefineNode
        : public epu::ZeroForEachFunctorI<RefineNode<REFINE> >
{
    typedef RefineNode<REFINE> this_type;

    typedef typename REFINE::value_type value_type;
    typedef value_type T;
    typedef typename REFINE::node_type node_type;
    typedef typename REFINE::dicer_type dicer_type;
    typedef typename REFINE::emit_info emit_info;
    typedef typename REFINE::emit_info_bcv emit_info_bcv;

    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<Imath::Vec3<T> > B3T;
    typedef Imath::Interval<T> IvlT;

    const V3i* InCellMins;
    const V3i* InCellMaxs;
    const dicer_type* Dicer;
    node_type* Node;

    emit_info_bcv* OutEmitted;

    void testChild( const Box3i& i_cbounds )
    {
        // Don't bother with diced rects that are outside the
        // original cell bounds.  This happens because of the way
        // we expand to create power-of-two dicing.
        if ( !Dicer->cellBounds().intersects( i_cbounds ) )
        {
            return;
        }

        const V3i csize = ( i_cbounds.max - i_cbounds.min ) + V3i( 1 );
        EMLD_DEBUG_ASSERT( IsValidGridSize( csize, Dicer->microGridSize() ),
                     "Bad grid subdiv size: " << csize );
        B3T obounds = Dicer->cellToObject( i_cbounds );

        // Convert to the 8-corners approach for node range evaluation.
        V3T corners[8];
        corners[0] = V3T( obounds.min.x, obounds.min.y, obounds.min.z );
        corners[1] = V3T( obounds.max.x, obounds.min.y, obounds.min.z );
        corners[2] = V3T( obounds.min.x, obounds.max.y, obounds.min.z );
        corners[3] = V3T( obounds.max.x, obounds.max.y, obounds.min.z );
        corners[4] = V3T( obounds.min.x, obounds.min.y, obounds.max.z );
        corners[5] = V3T( obounds.max.x, obounds.min.y, obounds.max.z );
        corners[6] = V3T( obounds.min.x, obounds.max.y, obounds.max.z );
        corners[7] = V3T( obounds.max.x, obounds.max.y, obounds.max.z );

        IvlT lsr = Node->range( corners );
        if ( lsr.intersects( T(0) ) )
        {
            OutEmitted->push_back( emit_info( i_cbounds, lsr ) );
        }
    }

    void subdivide( std::ptrdiff_t i )
    {
        const V3i cmin = InCellMins[i];
        const V3i cmax = InCellMaxs[i];
        const Box3i cbounds( cmin, cmax );
        const V3i csize = ( cmax - cmin ) + V3i( 1 );

        // Confirm that csize is a valid grid size.
        EMLD_DEBUG_ASSERT( IsValidGridSize( csize, Dicer->microGridSize() ),
                     "Bad grid subdiv size: " << csize );

        // Valid grid sizes are always (N-1)*(2^p) + 1  in size.
        V3i halfSize = V3i( 1 ) + ( ( csize - V3i( 1 ) ) / 2 );
        EMLD_DEBUG_ASSERT( IsValidGridSize( halfSize, Dicer->microGridSize() ),
                     "Bad half-grid subdiv size: " << halfSize );

        // Get midpoint.
        const V3i cmid = cmin + halfSize - V3i( 1 );
        EMLD_DEBUG_ASSERT( ( cmid + halfSize - V3i( 1 ) ) == cmax,
                     "Mismatched half size. cmin = " << cmin
                     << ", cmid = " << cmid << ", cmax = " << cmax );

        // Eight children.

        // 000
        testChild( Box3i( V3i( cmin.x, cmin.y, cmin.z ),
                   V3i( cmid.x, cmid.y, cmid.z ) ) );

        // 100
        testChild( Box3i( V3i( cmid.x, cmin.y, cmin.z ),
                   V3i( cmax.x, cmid.y, cmid.z ) ) );

        // 010
        testChild( Box3i( V3i( cmin.x, cmid.y, cmin.z ),
                   V3i( cmid.x, cmax.y, cmid.z ) ) );

        // 110
        testChild( Box3i( V3i( cmid.x, cmid.y, cmin.z ),
                   V3i( cmax.x, cmax.y, cmid.z ) ) );

        // 001
        testChild( Box3i( V3i( cmin.x, cmin.y, cmid.z ),
                   V3i( cmid.x, cmid.y, cmax.z ) ) );

        // 101
        testChild( Box3i( V3i( cmid.x, cmin.y, cmid.z ),
                   V3i( cmax.x, cmid.y, cmax.z ) ) );

        // 011
        testChild( Box3i( V3i( cmin.x, cmid.y, cmid.z ),
                   V3i( cmid.x, cmax.y, cmax.z ) ) );

        // 111
        testChild( Box3i( V3i( cmid.x, cmid.y, cmid.z ),
                   V3i( cmax.x, cmax.y, cmax.z ) ) );
    }

    void operator()( std::ptrdiff_t i ) const
    {
        const_cast<this_type*>( this )->subdivide( i );
    }
};

//-*****************************************************************************
template <typename REFINE>
struct DeinterleaveEmitInfo
        : public epu::ZeroForEachFunctorI<DeinterleaveEmitInfo<REFINE> >
{
    typedef DeinterleaveEmitInfo<REFINE> this_type;

    typedef typename REFINE::value_type value_type;
    typedef value_type T;
    typedef typename REFINE::emit_info emit_info;

    typedef Imath::Interval<T> IvlT;

    const emit_info* InEmitted;
    V3i* OutCellMins;
    V3i* OutCellMaxs;
    IvlT* OutLevelSetRange;

    void operator()( std::ptrdiff_t i ) const
    {
        const emit_info& e = InEmitted[i];
        OutCellMins[i] = e.cellBounds.min;
        OutCellMaxs[i] = e.cellBounds.max;
        OutLevelSetRange[i] = e.levelSetRange;
    }
};

//-*****************************************************************************
// This is an Array Functor which can be passed into the ParallelUtil
// reduction tools.  It turns the cell mins, cell maxs, and dicer into
// object bounds.
template <typename REFINE>
struct ObjectBoundsAF
    : public eu::AllZeroConstructor<ObjectBoundsAF<REFINE> >
{
    typedef typename REFINE::value_type T;
    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;
    typedef B3T value_type;
    typedef std::ptrdiff_t index_type;

    typedef typename REFINE::dicer_type dicer_type;

    const dicer_type* Dicer;
    const V3i* CellMins;
    const V3i* CellMaxs;

    B3T operator[]( std::ptrdiff_t i ) const
    {
        return Dicer->cellToObject( CellMins[i], CellMaxs[i] );
    }
};

//-*****************************************************************************
//-*****************************************************************************
// REFINERY CLASS - refines a node by some dicer into a mesh.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
Refinery<NODE,DICER,MESH>::Refinery( NODE& i_node,
                                     const DICER& i_dicer,
                                     MESH& o_mesh )
    : m_node( i_node )
    , m_dicer( i_dicer )
    , m_mesh( o_mesh )
{
    // Get initial cell grid from dicer.
    Box3i initGridBounds = m_dicer.initGridBounds();
    m_state.cellMins.push_back( initGridBounds.min );
    m_state.cellMaxs.push_back( initGridBounds.max );
    m_state.levelSetRange.push_back( IvlT( T(-1000), T(1000) ) );

    // Init bounds.
    m_bounds = m_dicer.cellToObject( initGridBounds );

    // Compute number of subdivs.
    m_state.subdivsRemaining = m_dicer.initGridSubdivs();
    if ( m_node.verbose() )
    {
        std::cout << "Number of subdivs remaining: "
                  << m_state.subdivsRemaining << std::endl;
    }

    // Set the mesh creation to false.
    m_state.createdMesh = false;
}

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
void Refinery<NODE,DICER,MESH>::recomputeBounds()
{
    if ( m_state.createdMesh )
    {
        m_bounds = m_mesh.bounds();
    }
    else
    {
        m_bounds.makeEmpty();

        typedef ObjectBoundsAF<this_type> OAF;
        typedef epu::BoundsBoundsReduceFunctor<B3T> RF;
        typedef epu::ValueReduceAdaptor<OAF,RF> VRA;

        std::size_t numCells = m_state.cellMins.size();
        if ( numCells > 0 )
        {
            OAF oaf;
            oaf.CellMins = eu::vector_cdata( m_state.cellMins );
            oaf.CellMaxs = eu::vector_cdata( m_state.cellMaxs );
            oaf.Dicer = &m_dicer;

            RF rf;

            VRA vra( oaf, rf );
            vra.execute( numCells );
            m_bounds = vra.ReducedValue;
        }
    }
}

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
void Refinery<NODE,DICER,MESH>::step()
{
    if ( m_state.createdMesh ) { return; }

    if ( m_state.subdivsRemaining > 0 )
    {
        stepRefineCells();
    }
    else
    {
        stepCreateMesh();
    }

    recomputeBounds();
}

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
void Refinery<NODE,DICER,MESH>::refine()
{
    // LOOP UNTIL MESH IS CREATED.
    while ( !m_state.createdMesh )
    {
        // Only subdivide if we can.
        if ( m_state.subdivsRemaining == 0 )
        {
            // Create mesh.
            stepCreateMesh();
        }
        else
        {
            // Refine cells.
            stepRefineCells();
        }
    }

    recomputeBounds();
}

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
void Refinery<NODE,DICER,MESH>::stepRefineCells()
{
    EMLD_ASSERT( !m_state.createdMesh &&
                 m_state.subdivsRemaining > 0,
                 "Should not have created mesh yet" );

    // Clear emit info, refine into it.
    std::size_t N = m_state.cellMins.size();
    m_emitInfoBCV.clear();
    if ( N > 0 )
    {
        RefineNode<this_type> F;
        F.InCellMins = eu::vector_cdata( m_state.cellMins );
        F.InCellMaxs = eu::vector_cdata( m_state.cellMaxs );
        F.Dicer = &m_dicer;
        F.OutEmitted = &m_emitInfoBCV;
        F.Node = &m_node;
        F.execute( N );
    }

    // De-interleave emit info back into state.
    std::size_t newN = m_emitInfoBCV.size();
    m_state.cellMins.resize( newN );
    m_state.cellMaxs.resize( newN );
    m_state.levelSetRange.resize( newN );
    if ( newN > 0 )
    {
        DeinterleaveEmitInfo<this_type> F;
        F.InEmitted = eu::vector_cdata( m_emitInfoBCV );
        F.OutCellMins = eu::vector_data( m_state.cellMins );
        F.OutCellMaxs = eu::vector_data( m_state.cellMaxs );
        F.OutLevelSetRange = eu::vector_data( m_state.levelSetRange );
        F.execute( newN );
    }

    // REPORT
    if ( m_node.verbose() )
    {
        std::cout << "Refined: " << N << " rects to create: " << newN
                  << " new ones" << std::endl;
    }

    // Decrease num subdivs.
    --( m_state.subdivsRemaining );
    if ( m_node.verbose() )
    {
        std::cout << "Num subdivs remaining: "
                  << m_state.subdivsRemaining << std::endl;
    }
}

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
void Refinery<NODE,DICER,MESH>::stepCreateMesh()
{
    // Should not have created mesh yet.
    EMLD_ASSERT( !m_state.createdMesh &&
                 m_state.subdivsRemaining == 0,
                 "Should not have created mesh yet" );

    // Turn cells into mesh.
    {
        ProcessCells<NODE,DICER,MESH> pc( m_state.cellMins,
                                          m_state.cellMaxs,
                                          m_node,
                                          m_dicer,
                                          m_mesh );
    }

    // Tell the state 'sup.
    m_state.createdMesh = true;
}

} // End namespace ElutMesh
} // End namespace EmldCore

#endif
