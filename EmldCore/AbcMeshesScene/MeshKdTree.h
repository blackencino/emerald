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

#ifndef _EmldCore_AbcMeshesScene_MeshKdTree_h_
#define _EmldCore_AbcMeshesScene_MeshKdTree_h_

#include "Foundation.h"
#include "Mesh.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
class MeshHandle
{
public:
    MeshHandle( Mesh& i_mesh ) : m_mesh( i_mesh ) {}
    virtual ~MeshHandle() {}

    Mesh& mesh() { return m_mesh; }
    const Mesh& mesh() const { return m_mesh; }

protected:
    Mesh& m_mesh;
};

//-*****************************************************************************
typedef ABCM_SHARED_PTR<MeshHandle> MeshHandleSptr;
typedef std::vector<MeshHandleSptr> MeshHandleSptrVec;

//-*****************************************************************************
// Finds closest points and does inside/outside tests.
// We want this to be as versatile as possible, so we assume no
// container semantics for the TriMeshes.
class MeshKdTree
{
public:
    struct MeshBounds
    {
        MeshBounds(){}
        inline const Box3d &operator()( const Mesh *m ) const
        {
            return m->simBounds();
        }
    };
    
    struct MeshSortPt
    {
        MeshSortPt(){}
        inline V3d operator()( const Mesh *m ) const
        {
            return m->simBounds().center();
        }
    };
    
    typedef EmldCore::SpatialSubd
        ::KdTree3<double, const Mesh *,
                    MeshBounds, MeshSortPt> Tree;
    
    MeshKdTree( MeshHandleSptrVec &i_vec );

    // Don't need a destructor.

    //-*************************************************************************
    // Region Aggregation Stuff
    //-*************************************************************************

    // Is the space inside a given set of bounds fully inside this region?
    // If we assume the mesh is closed - the triMesh intersection functions
    // return whether any of the triangles are inside the box. If the box
    // is not totally outside our bounds, and the box does not intersect
    // the mesh, then we just test the bounds center against the mesh inside.
    bool areBoundsFullyInside( const Box3d& i_bounds ) const;

    // Is the space inside a given set of bounds fully outside this region?
    bool areBoundsFullyOutside( const Box3d& i_bounds ) const;

    // Does the region intersect the bounds?
    // We treat this as an interior test.
    bool intersects( const Box3d& i_bounds ) const;

    // Does the region intersect the point?
    bool intersects( const V3d& i_point ) const;

    // Does the region intersect the point, with closest point info...
    bool intersects( const V3d& i_point,
                     BestMeshPointD& o_bestMeshPoint ) const;

    // Does a narrow interior band of the region intersect the bounds?
    // This is an optimization only, and by default just returns intersects.
    bool intersectsInnerNarrowBand( const Box3d& i_bounds,
                                    double i_narrowBand2 ) const;

    // Does a narrow interior band of the region intersect the point?
    bool intersectsInnerNarrowBand( const V3d& i_point,
                                    double i_narrowBand2,
                                    BestMeshPointD& o_bestMeshPoint ) const;

    //-*************************************************************************
    // END REGION AGGREGATION STUFF
    //-*************************************************************************

    Tree& tree() { return *m_tree; }
    const Tree& tree() const { return *m_tree; }
    
protected:
    std::vector<const Mesh *> m_meshes;
    ABCM_UNIQUE_PTR<Tree> m_tree;
};
    
} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif
