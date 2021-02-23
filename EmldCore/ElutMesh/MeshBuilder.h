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

#ifndef _EmldCore_ElutMesh_MeshBuilder_h_
#define _EmldCore_ElutMesh_MeshBuilder_h_

//-*****************************************************************************
// This is an implementation of the paper:
// P. Cignoni, F. Ganovelli, C. Montani, and R. Scopigno:
// Reconstruction of Topologically Correct and Adaptive Trilinear Isosurfaces,
// Computer & Graphics, Elsevier Science, 1999
// Which provides an exhaustive LUT to disambiguate isosurface triangulations
// inside a rectangular cell with iso values on the 8 corners of the cell,
// such that meshes will link together without having to explicitly march
// through them, as in Marching Cubes.
//-*****************************************************************************

#include "Foundation.h"
#include "Cell.h"
#include "PreVertex.h"
#include "PreTriangle.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
// MESH CONCEPT
//-*****************************************************************************
// The Mesh Builder below operates in its "bake" function on a MESH passed
// as a template parameter to the member function. This is to allow the
// use of this library with any mesh toolkit.  The mesh class provided
// must adhere to the following Mesh "concept".
//
template <typename T>
class MeshConcept
{
public:
    typedef T value_type;
    typedef MeshConcept<T> this_type;

    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;

    // This can be any container that matches std::vector semantics and
    // will respond to the vector_data and vector_cdata functions in the eu::
    // namespace
    typedef std::vector<V3i> V3i_vector_type;
    typedef std::vector<V3T> V3T_vector_type;

    // Clear the arrays of the mesh.
    void clear();

    // Access to the mesh's minimal arrays, as containers. The mesh
    // builder will resize these directly.
    V3T_vector_type& positions();
    const V3T_vector_type& positions() const;

    V3T_vector_type& dPdus();
    const V3T_vector_type& dPdus() const;

    V3T_vector_type& dPdvs();
    const V3T_vector_type& dPdvs() const;

    V3T_vector_type& dPdws();
    const V3T_vector_type& dPdws() const;

    V3T_vector_type& normals();
    const V3T_vector_type& normals() const;

    // The bounds will be recalculated by the mesh builder as well.
    void setBounds( const B3T& i_bounds );

    // The builder will call a "buildFinished" function to indicate
    // that the build is finished.
    void buildFinished();
};

//-*****************************************************************************
// This isn't really a mesh. It's a tool for producing mesh triangle indices
// and mesh vertex positions (and a vertex array), which allows for
// parallel construction.
template <typename NODE, typename DICER, typename MESH>
class MeshBuilder
{
public:
    typedef MeshBuilder<NODE,DICER,MESH> this_type;
    typedef NODE node_type;
    typedef DICER dicer_type;
    typedef MESH mesh_type;
    typedef Cell<this_type> cell_type;

    typedef typename NODE::value_type value_type;
    typedef value_type T;
    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;

    typedef epu::BucketedConcurrentVector<PreVertex<T> > PreVertexBCV;
    typedef epu::BucketedConcurrentVector<PreTriangle<T > > PreTriangleBCV;

    //-*************************************************************************
    // UNORDERED MAP TYPEDEF
    //-*************************************************************************
    typedef typename PreVertex<T>::HashF pv_hash_type;
    typedef typename PreVertex<T>::EqualF pv_equal_type;

    // The unordered map typedef.
#if EMLD_USE_CXX11
    typedef std::unordered_map<PreVertex<T>, int, 
                               pv_hash_type, pv_equal_type>
    HashMap;
#else
    typedef boost::unordered_map<PreVertex<T>, int, 
                                 pv_hash_type, pv_equal_type>
    HashMap;
#endif

    //-*************************************************************************
    // Mesh Builder is constructed from a const reference to a dicer, which
    // it will keep as a const reference. The dicer that is given here must
    // continue to exist while the mesh builder is in operation.
    MeshBuilder( NODE& io_node,
                 const DICER& i_dicer,
                 MESH& o_mesh ) 
        : m_node( io_node )
        , m_dicer( i_dicer )
        , m_mesh( o_mesh ) 
        {}

    //-*************************************************************************
    // This function can be called by threads in parallel to create 
    // pre-vertices and pre-triangles from a dense 3D tile of level set
    // data, occupying a Box3i of coordinates, with a border, in cell
    // coordinate space.
    void gatherPreTriangles( 

        // The bounds, in coordinate space, of the level set data grid
        // passed below. These bounds INCLUDE the border.
        const Box3i& i_tileBounds,

        // The border, which is assumed to be the same on all six sides.
        // this is extra data which is elsewhere used for extrapolation
        // or synchronization. We don't use it here, we just respect it when
        // iterating over the cell data.
        int i_border,

        // The level set data, fully occupying the tile bounds described
        // above. It is assumed that the array is ordered with the z (k)
        // coordinate as major-major, the y (j) coordinate as major, and
        // the x (i) coordinate as minor.
        const T* i_levelSets,

        // The position and filter axis data, strided as the level sets are.
        const V3T* i_p,
        const V3T* i_dPdu,
        const V3T* i_dPdv,
        const V3T* i_dPdw );

    //-*************************************************************************
    // This function cannot be called in parallel, and must only be called
    // after all the gathering of pre-triangles above is complete.
    // It will bake the pre vertices & pre triangles into vertex positions
    // & triangle data.
    void finishMesh();

    //-*************************************************************************
    // Called to transform a point from cell into world.
    //V3T cellToWorld( const V3i& i_cell ) const 
    //{ return m_transform( i_cell ); }
    //const DICER& dicer() const { return m_dicer; }

    //-*************************************************************************
    // Called by cell, which is called by our gatherPreTriangles function.
    void emitPreTriangle
    (
        const PreVertex<T>& i_vert0,
        const PreVertex<T>& i_vert1,
        const PreVertex<T>& i_vert2
    )
    {
        m_preVertices.push_back( i_vert0 );
        m_preVertices.push_back( i_vert1 );
        m_preVertices.push_back( i_vert2 );
        m_preTriangles.push_back( PreTriangle<T>( i_vert0, i_vert1, i_vert2 ) );
    }

protected:
    // DATA!
    NODE& m_node;
    const DICER& m_dicer;
    MESH& m_mesh;

    // Pre-stuff
    PreVertexBCV m_preVertices;
    PreTriangleBCV m_preTriangles;

    // Hash map.
    HashMap m_hashMap;
};

//-*****************************************************************************
// What are we iterating in parallel over? This library is about an approach
// to doing marching cubes, so the iteration is necessarily over some grid
// space. I think it's sufficiently general to suppose that there's a
// pre-defined integer coordinate space that is global, and that the triangles
// and vertices are created relative to this coordinate space. There's some
// mapping between the integer coordinate space and a global cartesian
// coordinate space, which could be frustum-aligned, polar, who knows, it's
// externally supplied.
// Our immediate use for this library will be to iterate over dense 3D tiles of
// the integer coordinate space. These tiles will eventually have some border.
// We assume these tiles are expensive to compute, and that they probably
// won't be saved globally.  So, some external process is going to call, in
// parallel, a member function of our mesher which accepts a tile in the
// integer coordinate space, along with an array of computed level-set values.
// This member function of our mesher will do the first pass of creating
// (duplicate) pre-vertices and pre-triangles.
// Let's call this function gatherPreVertsAndTris.
template <typename NODE, typename DICER, typename MESH>
void MeshBuilder<NODE,DICER,MESH>::gatherPreTriangles
(
    const Box3i& i_tileBounds,
    int i_border,
    const T* i_levelSets,
    const V3T* i_p,
    const V3T* i_dPdu,
    const V3T* i_dPdv,
    const V3T* i_dPdw
)
{
    const V3i tileSize = V3i( 1 ) + ( i_tileBounds.max - i_tileBounds.min );
    int JSTRIDE = tileSize.x;
    int KSTRIDE = tileSize.x * tileSize.y;

    const Box3i innerTile( i_tileBounds.min + V3i( i_border ),
                           i_tileBounds.max - V3i( i_border ) );
#ifdef DEBUG
    EMLD_ASSERT( innerTile.max.z > innerTile.min.z &&
                 innerTile.max.y > innerTile.min.y &&
                 innerTile.max.x > innerTile.min.x,
                 "Inner tile is degenerate. Inner tile: " 
                 << innerTile << ", tile bounds: " 
                 << i_tileBounds << ", border = " << i_border );
#endif

    // Loop over the cells of the inner tile, produce cells.
    for ( int k = innerTile.min.z; k < innerTile.max.z; ++k )
    {
        int lsk = k - i_tileBounds.min.z;
        for ( int j = innerTile.min.y; j < innerTile.max.y; ++j )
        {
            int lsj = j - i_tileBounds.min.y;
            for ( int i = innerTile.min.x; i < innerTile.max.x; ++i )
            {
                int lsi = i - i_tileBounds.min.x;

                std::size_t offset = ( KSTRIDE * lsk ) +
                              ( JSTRIDE * lsj ) +
                              ( lsi );

                // Make a cell. It will check the level sets, and if
                // there's a zero crossing, emit pre-verts and pre-tris.
                cell_type cell( *this,          // Pointer to mesh
                                JSTRIDE,        // Strides
                                KSTRIDE,
                                V3i( i, j, k ), // Lower left coordinate

                                // Arrays, offset to start.
                                i_levelSets + offset,
                                i_p + offset,
                                i_dPdu + offset,
                                i_dPdv + offset, 
                                i_dPdw + offset );
            }
        }
    }
}

//-*****************************************************************************
// Helper functor for vertex baking
template <typename MBUILD>
struct VertexBake :
    public epu::ZeroForEachFunctorI<VertexBake<MBUILD> >
{
    typedef typename MBUILD::value_type value_type;
    typedef value_type T;
    typedef Imath::Vec3<T> V3T;

    const PreVertex<T>* PreVertices;
    V3T* Positions;
    V3T* DpDus;
    V3T* DpDvs;
    V3T* DpDws;

    void operator()( std::ptrdiff_t i ) const
    {
        PreVertices[i].get( Positions[i],
                            DpDus[i], DpDvs[i], DpDws[i] );
    }
};

//-*****************************************************************************
// Helper functor for triangle baking
template <typename MBUILD>
struct TriangleBake :
    public epu::ZeroForEachFunctorI<TriangleBake<MBUILD> >
{
    typedef typename MBUILD::value_type value_type;
    typedef value_type T;
    typedef typename MBUILD::HashMap hash_map;

    const hash_map* HashMap;
    const PreTriangle<T>* PreTriangles;
    V3i* TriIndices;

    int vertexIndex( const PreVertex<T>& i_vtx ) const
    {
        // The "at" operator will throw an std::out_of_range exception on
        // a bad key. Just let it throw.
        return HashMap->at( i_vtx );
    }

    void operator()( std::ptrdiff_t i ) const
    {
        const PreTriangle<T>& pt = PreTriangles[i];

        V3i tri = V3i( HashMap->at( pt.vertex0() ),
                       HashMap->at( pt.vertex1() ),
                       HashMap->at( pt.vertex2() ) );
        TriIndices[i] = tri;
    }
};

//-*****************************************************************************
// Helper function for normals
template <typename MBUILD>
struct ComputeNormals
        : public epu::ZeroForEachFunctor<ComputeNormals<MBUILD> >
{
    typedef typename MBUILD::value_type value_type;
    typedef value_type T;
    typedef typename MBUILD::node_type node_type;
    typedef Imath::Vec3<T> V3T;

    const V3T* InPositions;
    const V3T* InDpDus;
    const V3T* InDpDvs;
    const V3T* InDpDws;

    node_type* Node;
    V3T* OutNormals;

    // virtual void GradientEvalMultipleFiltered(int neval, RtPoint *result,
    //     const RtPoint *p, const RtPoint *dPdu, const RtPoint *dPdv,
    //     const RtPoint *dPdw) {
    //     GradientEvalMultiple(neval, result, p);
    // }

    void operator()( std::ptrdiff_t i_begin,
                     std::ptrdiff_t i_end ) const
    {
        node_type* nonConstNode = const_cast<node_type*>( Node );
        V3T* nonConstNormals = const_cast<V3T*>( OutNormals );

        // CJH HACK: this granularity is not carefully tested
        static const std::ptrdiff_t GRAN = 256;
        std::ptrdiff_t b = i_begin;
        std::ptrdiff_t e = std::min( b + GRAN, i_end );
        while ( b < i_end )
        {
            std::size_t numPts = e - b;
            nonConstNode->gradientMultipleFiltered( 
                numPts,
                nonConstNormals + b,
                InPositions + b,
                InDpDus + b,
                InDpDvs + b,
                InDpDws + b );

            b = e;
            e = std::min( e + GRAN, i_end );
        }

        // Normalize, for now.
        // CJH HACK: Not sure whether it's better to leave alone.
        for ( std::ptrdiff_t i = i_begin; i < i_end; ++i )
        {
            nonConstNormals[i].normalize();
        }
    }
};

//-*****************************************************************************
// De-duplicate vertices, put them into an unordered map. Then make triangles
// by mapping the pre-triangles to the de-duplicated vertices.
// IN: BCV<PreVertex> preVerts;
//     BCV<PreTriangle> preTris;
// OUT: V<V3T> positions;
//      V<V3T> dPdu;
//      V<V3T> dPdv;
//      V<V3T> dPdw;
//      V<V3T> normals;
//      V<V3i> triangleIndices;
template <typename NODE, typename DICER, typename MESH>
void MeshBuilder<NODE,DICER,MESH>::finishMesh()
{
    // Remove duplicates from pre vertices.
    epu::VectorRemoveDuplicatesInPlaceEqual( m_preVertices );
    std::size_t numVerts = m_preVertices.size();

    // Resize actual vertices and vertex data.
    m_mesh.clear();
    m_mesh.positions().resize( numVerts );
    m_mesh.dPdus().resize( numVerts );
    m_mesh.dPdvs().resize( numVerts );
    m_mesh.dPdws().resize( numVerts );
    m_mesh.normals().resize( numVerts );

    // Bake vertex positions.
    if ( numVerts > 0 )
    {
        VertexBake<this_type> F;
        F.PreVertices = eu::vector_cdata( m_preVertices );
        F.Positions = eu::vector_data( m_mesh.positions() );
        F.DpDus = eu::vector_data( m_mesh.dPdus() );
        F.DpDvs = eu::vector_data( m_mesh.dPdvs() );
        F.DpDws = eu::vector_data( m_mesh.dPdws() );
        F.execute( numVerts );
    }

    // Resize triangle indices.
    std::size_t numTris = m_preTriangles.size();
    m_mesh.triIndices().resize( numTris );

    // This has to be done serially. Insert indices, keyed off of pre-vertex,
    // into hash map.
    if ( numVerts > 0 )
    {
        m_hashMap.clear();

        // Set num buckets to num verts * 2, which should avoid most collisions.
        m_hashMap.rehash( numVerts * 2 );

        EMLD_ASSERT( numVerts < std::numeric_limits<int>::max(),
                    "Can't fit " << numVerts 
                    << " vertices in 32-bit int range" );

        // Loop over verts, add them to hash map.
        for ( int i = 0; i < numVerts; ++i )
        {
            m_hashMap[ m_preVertices[i] ] = i;
        }
    }

    // Bake triangle indices.
    if ( numTris > 0 )
    {
        TriangleBake<this_type> F;
        F.HashMap = &m_hashMap;
        F.PreTriangles = eu::vector_cdata( m_preTriangles );
        F.TriIndices = eu::vector_data( m_mesh.triIndices() );
        F.execute( numTris );
    }

    // Compute normals.
    if ( numVerts > 0 )
    {
        ComputeNormals<this_type> F;
        F.InPositions = eu::vector_cdata( m_mesh.positions() );
        F.InDpDus = eu::vector_cdata( m_mesh.dPdus() );
        F.InDpDvs = eu::vector_cdata( m_mesh.dPdvs() );
        F.InDpDws = eu::vector_cdata( m_mesh.dPdws() );
        F.Node = &m_node;
        F.OutNormals = eu::vector_data( m_mesh.normals() );
        F.execute( numVerts );
    }

    // Recompute bounds.
    B3T pbnds;
    pbnds.makeEmpty();
    if ( numVerts > 0 )
    {
        pbnds = epu::VectorVecBounds( m_mesh.positions() );
    }
    m_mesh.setBounds( pbnds );

    // Inform mesh that it is finished.
    m_mesh.buildFinished();
}

} // End namespace ElutMesh
} // End namespace EmldCore

#endif

