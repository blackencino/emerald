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

#include "MeshKdTree.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
struct BoundsFullyInsideAny
{
    BoundsFullyInsideAny( const Box3d& i_bounds )
        : testBounds( i_bounds )
        , foundMesh( NULL )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        return i_bnds.intersects( testBounds );
    }

    void leaf( const Mesh* i_mesh )
    {
        if ( !foundMesh && i_mesh->areBoundsFullyInside( testBounds ) )
        {
            foundMesh = i_mesh;
        }
    }

    bool unfinished() const { return !foundMesh; }
    bool found() const { return ( bool )foundMesh; }

    Box3d testBounds;
    const Mesh* foundMesh;
};

//-*****************************************************************************
struct BoundsNotFullyOutsideAny
{
    BoundsNotFullyOutsideAny( const Box3d& i_bounds )
        : testBounds( i_bounds )
        , foundMesh( NULL )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        return i_bnds.intersects( testBounds );
    }

    void leaf( const Mesh* i_mesh )
    {
        if ( !foundMesh &&
             !( i_mesh->areBoundsFullyOutside( testBounds ) ) )
        {
            foundMesh = i_mesh;
        }
    }

    bool unfinished() const { return !foundMesh; }
    bool found() const { return ( bool )foundMesh; }

    Box3d testBounds;
    const Mesh* foundMesh;
};

//-*****************************************************************************
struct BoundsIntersectAny
{
    BoundsIntersectAny( const Box3d& i_bounds )
        : testBounds( i_bounds )
        , foundMesh( NULL )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        return i_bnds.intersects( testBounds );
    }

    void leaf( const Mesh* i_mesh )
    {
        if ( !foundMesh && i_mesh->intersects( testBounds ) )
        {
            foundMesh = i_mesh;
        }
    }

    bool unfinished() const { return !foundMesh; }
    bool found() const { return ( bool )foundMesh; }

    Box3d testBounds;
    const Mesh* foundMesh;
};

//-*****************************************************************************
struct PointIntersectAny
{
    PointIntersectAny( const V3d& i_point )
        : testPoint( i_point )
        , foundMesh( NULL )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        return i_bnds.intersects( testPoint );
    }

    void leaf( const Mesh* i_mesh )
    {
        if ( !foundMesh && i_mesh->intersects( testPoint ) )
        {
            foundMesh = i_mesh;
        }
    }

    bool unfinished() const { return !foundMesh; }
    bool found() const { return ( bool )foundMesh; }

    V3d testPoint;
    const Mesh* foundMesh;
};

//-*****************************************************************************
struct BestMeshPointTraverse : public BestMeshPointD
{
    BestMeshPointTraverse( const V3d& i_p )
        : BestMeshPointD( i_p )
    {}

    BestMeshPointTraverse( const V3d& i_p,
                           double i_maxD2 )
        : BestMeshPointD( i_p, i_maxD2 )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        // Expand the bounds _slightly_, to make sure that roundoff
        // error doesn't nuke it.
        return EmldCore::SpatialSubd
               ::PointBoxMinimumSquaredDistanceLessThan(
                   simPoint, i_bnds, 1.01 * bestSquaredDist );
    }

    void leaf( const Mesh* i_mesh )
    {
        BestMeshPointD bp = ( *this );
        if ( i_mesh->intersectsInnerNarrowBand( simPoint,
                                                bestSquaredDist,
                                                bp ) )
        {
            if ( bp.bestSquaredDist < bestSquaredDist )
            {
                reinterpret_cast<BestMeshPointD&>( *this ) = bp;
            }
        }
    }

    // Always need to keep checking for closest point stuff.
    bool unfinished() const { return true; }

    bool found() const
    {
        return ( bestTriangle != NULL ) &&
               ( bestMesh != NULL ) &&
               ( bestSquaredDist <= maxSquaredDist );
    }
};

//-*****************************************************************************
struct BoundsIntersectInnerNarrowBandAny
{
    BoundsIntersectInnerNarrowBandAny( const Box3d& i_bounds, double i_r2 )
        : testBounds( i_bounds )
        , narrowBand2( i_r2 )
        , foundMesh( NULL )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        return i_bnds.intersects( testBounds );
    }

    void leaf( const Mesh* i_mesh )
    {
        if ( !foundMesh &&
             i_mesh->intersectsInnerNarrowBand( testBounds,
                                                narrowBand2 ) )
        {
            foundMesh = i_mesh;
        }
    }

    bool unfinished() const { return !foundMesh; }
    bool found() const { return ( bool )foundMesh; }

    Box3d testBounds;
    double narrowBand2;
    const Mesh* foundMesh;
};

//-*****************************************************************************
struct PointIntersectInnerNarrowBand : public BestMeshPointD
{
    PointIntersectInnerNarrowBand( const V3d& i_p,
                                   double i_maxD2 )
        : BestMeshPointD( i_p, i_maxD2 )
        , tooDeep( false )
    {}

    bool validBounds( const Box3d& i_bnds ) const
    {
        if ( tooDeep ) { return false; }
        
        // Expand the bounds _slightly_, to make sure that roundoff
        // error doesn't nuke it.
        return EmldCore::SpatialSubd
               ::PointBoxMinimumSquaredDistanceLessThan(
                   simPoint, i_bnds, 1.01 * bestSquaredDist );
    }

    void leaf( const Mesh* i_mesh )
    {
        if ( tooDeep ) { return; }

        // First check inside.
        if ( !i_mesh->intersects( simPoint ) ) { return; }

        // We are definitely inside the mesh. If we are NOT
        // inside the narrow band, it means we're too deep inside,
        // and the test point is bad.

        // Now check the inner narrow band. Testing against maxSquared
        // distance, to find points that are too deep.
        BestMeshPointD bp = ( *this );
        if ( i_mesh->intersectsInnerNarrowBand( simPoint,
                                                maxSquaredDist,
                                                bp ) )
        {
            if ( bp.bestSquaredDist < bestSquaredDist )
            {
                reinterpret_cast<BestMeshPointD&>( *this ) = bp;
            }
        }
        else
        {
            tooDeep = true;
            bestTriangle = NULL;
            bestMesh = NULL;
            triId = -1;
            meshId = -1;
        }
    }

    // If too deep, we're finished.
    bool unfinished() const { return !tooDeep; }

    bool found() const
    {
        return ( !tooDeep ) &&
               ( bestTriangle != NULL ) &&
               ( bestMesh != NULL ) &&
               ( triId >= 0 ) &&
               ( meshId >= 0 ) &&
               ( bestSquaredDist <= maxSquaredDist );
    }

    bool tooDeep;
};

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// MESH KD TREE
//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
MeshKdTree::MeshKdTree( MeshHandleSptrVec& i_vec )
{
    // This lets me trick boost pointers.
    for ( MeshHandleSptrVec::iterator iter = i_vec.begin();
          iter != i_vec.end(); ++iter )
    {
        const Mesh* tm = &( ( *iter )->mesh() );
        m_meshes.push_back( tm );
    }

    MeshBounds mb;
    MeshSortPt ms;
    m_tree.reset( new Tree( m_meshes, mb, ms ) );
}

//-*****************************************************************************
// REGION STUFF
//-*****************************************************************************

//-*****************************************************************************
bool MeshKdTree::areBoundsFullyInside( const Box3d& i_bounds ) const
{
    BoundsFullyInsideAny b( i_bounds );
    m_tree->constTraverse( b );
    return b.found();
}

//-*****************************************************************************
bool MeshKdTree::areBoundsFullyOutside( const Box3d& i_bounds ) const
{
    BoundsNotFullyOutsideAny b( i_bounds );
    m_tree->constTraverse( b );
    return b.found();
}

//-*****************************************************************************
bool MeshKdTree::intersects( const Box3d& i_bounds ) const
{
    BoundsIntersectAny b( i_bounds );
    m_tree->constTraverse( b );
    return b.found();
}

//-*****************************************************************************
bool MeshKdTree::intersects( const V3d& i_point ) const
{
    PointIntersectAny p( i_point );
    m_tree->constTraverse( p );
    return p.found();
}

//-*****************************************************************************
// Does the region intersect the point, with closest point info...
bool MeshKdTree::intersects( const V3d& i_point,
                             BestMeshPointD& o_meshPoint ) const
{
    BestMeshPointTraverse trv( i_point );
    m_tree->constTraverse( trv );
    if ( trv.found() )
    {
        o_meshPoint = trv;
        return true;
    }
    else
    {
        return false;
    }
}

//-*****************************************************************************
// This is an optimization only - here we're returning whether we intersect
// any of the inner narrow bands, but we don't worry about being more accurate
bool MeshKdTree::intersectsInnerNarrowBand( const Box3d& i_bounds,
                                            double i_narrowBand2 ) const
{
    BoundsIntersectInnerNarrowBandAny b( i_bounds, i_narrowBand2 );
    m_tree->constTraverse( b );
    return b.found();
}

//-*****************************************************************************
// Does a narrow interior band of the region intersect the point?
bool MeshKdTree::intersectsInnerNarrowBand( const V3d& i_point,
                                            double i_narrowBand2,
                                            BestMeshPointD& o_meshPoint ) const
{
    PointIntersectInnerNarrowBand p( i_point, i_narrowBand2 );
    m_tree->constTraverse( p );
    if ( p.found() )
    {
        o_meshPoint = p;
        return true;
    }
    else
    {
        return false;
    }
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

