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

#ifndef _EmldCore_ElutMesh_ProcessCells_h_
#define _EmldCore_ElutMesh_ProcessCells_h_

#include "Foundation.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
class ProcessCells
{
public:
    typedef ProcessCells<NODE, DICER, MESH> this_type;
    typedef NODE node_type;
    typedef DICER dicer_type;
    typedef MESH mesh_type;
    typedef typename NODE::value_type value_type;
    typedef value_type T;

    typedef MeshBuilder<NODE, DICER, MESH> mesh_builder_type;

    ProcessCells
    (
        const V3iArray& i_cellMins,
        const V3iArray& i_cellMaxs,

        NODE& io_node,
        const DICER& i_dicer,
        MESH& o_mesh
    );
};

//-*****************************************************************************
template <typename PCELLS>
struct TestCells
        : public epu::ZeroForEachFunctor<TestCells<PCELLS> >
{
    typedef TestCells<PCELLS> this_type;
    typedef typename PCELLS::value_type value_type;
    typedef value_type T;
    typedef typename PCELLS::node_type node_type;
    typedef typename PCELLS::dicer_type dicer_type;
    typedef typename PCELLS::mesh_builder_type mesh_builder_type;

    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;

    const V3i* InCellMins;
    const V3i* InCellMaxs;

    node_type* Node;
    const dicer_type* Dicer;
    mesh_builder_type* MeshBuilder;

    //-*************************************************************************
    void testCell( const Box3i& i_cellBounds,
                   int i_NCUBE,
                   std::size_t i_numVerts,
                   T* o_levelSets,
                   V3T* o_pobjs,
                   V3T* o_dPdus,
                   V3T* o_dPdvs,
                   V3T* o_dPdws )
    {
#ifdef DEBUG
        V3i dcell = i_cellBounds.max - i_cellBounds.min + V3i( 1 );
        EMLD_DEBUG_ASSERT( dcell.x == i_NCUBE &&
                           dcell.y == i_NCUBE &&
                           dcell.z == i_NCUBE, "Bad frustum dicing." );
#endif

        // make a grid of NCUBE^3 points in cell space, and convert to world
        // space.
        int idx = 0;
        for ( int k = 0; k < i_NCUBE; ++k )
        {
            for ( int j = 0; j < i_NCUBE; ++j )
            {
                for ( int i = 0; i < i_NCUBE; ++i, ++idx )
                {
                    V3i pcell( i_cellBounds.min.x + i,
                               i_cellBounds.min.y + j,
                               i_cellBounds.min.z + k );

                    EMLD_DEBUG_ASSERT(
                        i_cellBounds.intersects( pcell ),
                        "Bad cell point: " << pcell );

                    Dicer->cellToObjectFilterAxes( pcell,
                                                   o_pobjs[idx],
                                                   o_dPdus[idx],
                                                   o_dPdvs[idx],
                                                   o_dPdws[idx] );
                }
            }
        }

        // eval the node on the multiple filtered points
        Node->evalMultipleFiltered( i_numVerts,
                                    o_levelSets,
                                    1, // result stride
                                    o_pobjs,
                                    o_dPdus,
                                    o_dPdvs,
                                    o_dPdws );

        // pass the voxel grid to the mesh builder.
        MeshBuilder->gatherPreTriangles( i_cellBounds, 0,
                                         o_levelSets,
                                         o_pobjs,
                                         o_dPdus,
                                         o_dPdvs,
                                         o_dPdws );
    }

    //-*************************************************************************
    void operator()( std::ptrdiff_t i_begin, std::ptrdiff_t i_end ) const
    {
        const int NCUBE = Dicer->microGridSize();

        const std::size_t numVerts = NCUBE * NCUBE * NCUBE;

        std::vector<V3f> pobjs( numVerts );
        std::vector<V3f> dPdus( numVerts );
        std::vector<V3f> dPdvs( numVerts );
        std::vector<V3f> dPdws( numVerts );
        std::vector<float> levelSets( numVerts );

        V3f* podata = eu::vector_data( pobjs );
        V3f* dPdudata = eu::vector_data( dPdus );
        V3f* dPdvdata = eu::vector_data( dPdvs );
        V3f* dPdwdata = eu::vector_data( dPdws );
        float* lsdata = eu::vector_data( levelSets );

        this_type* nonConstThis = const_cast<this_type*>( this );

        for ( std::ptrdiff_t i = i_begin; i != i_end; ++i )
        {
            nonConstThis->testCell( Box3i( InCellMins[i],
                                           InCellMaxs[i] ),
                                    NCUBE,
                                    numVerts,
                                    lsdata,
                                    podata,
                                    dPdudata,
                                    dPdvdata,
                                    dPdwdata );
        }
    }
};

//-*****************************************************************************
//-*****************************************************************************
// PROCESS CELLS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename NODE, typename DICER, typename MESH>
ProcessCells<NODE, DICER, MESH>::ProcessCells
(
    const V3iArray& i_cellMins,
    const V3iArray& i_cellMaxs,
    NODE& io_node,
    const DICER& i_dicer,
    MESH& o_mesh
)
{
    o_mesh.clear();

    std::size_t Ncells = i_cellMins.size();
    if ( Ncells > 0 )
    {
        // Make mesher
        mesh_builder_type meshBuilder( io_node, i_dicer, o_mesh );

        // Gather pre-triangles.
        {
            TestCells<this_type> F;
            F.Node = &io_node;
            F.InCellMins = eu::vector_cdata( i_cellMins );
            F.InCellMaxs = eu::vector_cdata( i_cellMaxs );
            F.Dicer = &i_dicer;
            F.MeshBuilder = &meshBuilder;
            F.execute( Ncells );
        }
        if ( io_node.verbose() )
        {
            std::cout << "Gathered pre-triangles from " << Ncells
                      << " cells." << std::endl;
        }

        // Bake mesher into final mesh.
        meshBuilder.finishMesh();
        if ( io_node.verbose() )
        {
            std::cout << "Baked pre-triangles into mesh."
                      << std::endl;
        }
    }
    else
    {
        // Let mesh know we're done with it, because builder above did not.
        o_mesh.buildFinished();
    }

    // Some reporting.
    if ( io_node.verbose() )
    {
        std::size_t Nverts = o_mesh.positions().size();
        std::size_t Ntris = o_mesh.triIndices().size();
        std::cout << "Evaluated " << Ncells << " cells to create: "
                  << Nverts << " vertices, and "
                  << Ntris << " triangles." << std::endl;
    }
}

} // End namespace ElutMesh
} // End namespace EmldCore

#endif
