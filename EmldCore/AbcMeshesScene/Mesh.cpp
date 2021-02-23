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

#include "Mesh.h"
#include "Scene.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
void Mesh::extractScale( bool i_set )
{
    // Get the scale out of the transformation matrix.
    M44d objectLocalToSim = m_enclosingObject.localToSim();

    // What is the scaling going on?
    V3d scl( 1.0 );
    V3d shr( 0.0 );
    if ( !Imath::extractAndRemoveScalingAndShear(
             objectLocalToSim, scl, shr, false ) )
    {
        ABCM_THROW( "Unable to extract scale and shear from transform" );
    }
    if ( i_set )
    {
        ABCM_ASSERT( scl.x != 0.0 && scl.y != 0.0 && scl.z != 0.0,
                     "Degenerate scale." );
        m_scale = scl;

        // So the scale is applied to the geometry.
        // InternalToLocal divides the scale back out
        // LocalToInternal multiplies the scale.
        m_internalToLocal.setScale( V3d( 1.0 / m_scale.x,
                                         1.0 / m_scale.y,
                                         1.0 / m_scale.z ) );
        m_localToInternal.setScale( m_scale );
    }
    else
    {
        // If not setting, then checking.
        const V3d diff = scl - m_scale;
        ABCM_ASSERT( std::abs( diff.x ) < 0.01 &&
                     std::abs( diff.y ) < 0.01 &&
                     std::abs( diff.z ) < 0.01,
                     "Cannot support animating scale." );
    }
}

//-*****************************************************************************
Mesh::Mesh( Object& i_enclosingObject,
            AbcG::IPolyMesh& i_abcPolyMesh,
            Scene& i_scene )
    : Object::Internal( i_enclosingObject )
    , m_abcPolyMesh( i_abcPolyMesh )
    , m_meshId( 0 )
{
    // Extract the scale and set the scale matrices.
    extractScale( true );

    // Set the variance.
    m_abcPolyMeshVariance = m_abcPolyMesh.getSchema().getTopologyVariance();

    // Get time range.
    if ( !m_abcPolyMesh.getSchema().isConstant() )
    {
        AbcG::TimeSamplingPtr iTsmp =
            m_abcPolyMesh.getSchema().getTimeSampling();

        size_t numSamps =  m_abcPolyMesh.getSchema().getNumSamples();
        if ( numSamps > 0 )
        {
            m_internalMinTime = iTsmp->getSampleTime( 0 );
            m_internalMaxTime = iTsmp->getSampleTime( numSamps - 1 );
        }
    }

    i_scene.addMeshHandle( *this );
}

//-*****************************************************************************
void Mesh::setTime()
{
    // Constant topology does not allow for animated scales.
    const bool isConstant =
        ( m_abcPolyMeshVariance == AbcG::kConstantTopology );

    // We set scale for non-constants.
    const bool doSetScale = !isConstant;

    extractScale( doSetScale );
}

//-*****************************************************************************
// REGION STUFF
//-*****************************************************************************

//-*****************************************************************************
// Is the mesh static?
bool Mesh::isStatic() const
{
    return ( m_abcPolyMeshVariance == AbcG::kConstantTopology ) &&
           !m_enclosingObject.isXformAnimating();
}

//-*****************************************************************************
// Is the region rigid?
bool Mesh::isRigid() const
{
    return ( m_abcPolyMeshVariance == AbcG::kConstantTopology );
}

//-*****************************************************************************
// Is the region consistent (topologically)?
bool Mesh::isConsistent() const
{
    return ( m_abcPolyMeshVariance == AbcG::kConstantTopology ) ||
           ( m_abcPolyMeshVariance == AbcG::kHomogenousTopology );
}

//-*****************************************************************************
// Is the region consistent (topologically)?
bool Mesh::isConsistentAndNotRigid() const
{
    return ( m_abcPolyMeshVariance == AbcG::kHomogenousTopology );
}

//-*****************************************************************************
V3d Mesh::transformShapePointToSim( const V3d& i_shapePoint ) const
{
    return i_shapePoint * shapeToSim();
}

//-*****************************************************************************
V3d Mesh::transformBarycentricToSim( const V3d& i_barycentric,
                                     int i_triIndex ) const
{
    const Etm::Triangle* tri = &( *( m_triMesh->triangle( i_triIndex ) ) );

    const V3d shapePoint(
        tri->barycentricEval( V2f( i_barycentric.x,
                                   i_barycentric.y ),
                              i_barycentric.z ) );

    return shapePoint * shapeToSim();
}

//-*****************************************************************************
// Is the space inside a given set of bounds fully inside this region?
// If we assume the mesh is closed - the triMesh intersection functions
// return whether any of the triangles are inside the box. If the box
// is not totally outside our bounds, and the box does not intersect
// the mesh, then we just test the bounds center against the mesh inside.
bool Mesh::areBoundsFullyInside( const Box3d& i_bounds ) const
{
    // First check all the way outside.
    if ( !simBounds().intersects( i_bounds ) ) { return false; }

    // Transform bounds internal
    Box3f internalBounds =
        toBox3f( Imath::transform( i_bounds, simToShape() ) );

    // Test against tri mesh - this is looking for surface intersection,
    // not interior.
    if ( m_triMesh->intersects( internalBounds ) ) { return false; }

    // Test whether the bounds center point is inside.
    return m_triMesh->isInside( internalBounds.center() );
}

//-*****************************************************************************
// Is the space inside a given set of bounds fully outside this region?
bool Mesh::areBoundsFullyOutside( const Box3d& i_bounds ) const
{
    // First check bounds.
    if ( !simBounds().intersects( i_bounds ) ) { return true; }

    // Transform bounds internal
    Box3f internalBounds = toBox3f(
                               Imath::transform( i_bounds, simToShape() ) );

    // Test against tri mesh - this is looking for surface intersection,
    // not interior. Any intersection means bounds overlaps surface, so
    // false on all the way outside.
    if ( m_triMesh->intersects( internalBounds ) ) { return false; }

    // Test whether the bounds center point is inside.
    return !( m_triMesh->isInside( internalBounds.center() ) );
}

//-*****************************************************************************
// Does the region intersect the bounds?
// We treat this as an interior test.
bool Mesh::intersects( const Box3d& i_bounds ) const
{
    // First check bounds.
    if ( !simBounds().intersects( i_bounds ) ) { return false; }

    // Transform bounds internal
    Box3f internalBounds = toBox3f(
                               Imath::transform( i_bounds, simToShape() ) );

    // Test against tri mesh - this is looking for surface intersection,
    // not interior. Any intersection means bounds overlaps surface, so
    // true on intersection.
    if ( m_triMesh->intersects( internalBounds ) ) { return true; }

    // If we get here, the bounds intersected in sim space,
    // and the transformed bounds did not intersect in internal space.
    // therefore, the bounds are either entirely inside the mesh or
    // entirely outside, but with some bounds overlap. So we test the
    // center point.
    return m_triMesh->isInside( internalBounds.center() );
}

//-*****************************************************************************
// Does the region intersect the point?
bool Mesh::intersects( const V3d& i_point,
                       V3d& o_localPoint ) const
{
    // First check bounds.
    if ( !simBounds().intersects( i_point ) ) { return false; }

    // Transform point internal
    V3f internalPoint = V3f( ( i_point * simToShape() ) );

    if ( m_triMesh->isInside( internalPoint ) )
    {
        o_localPoint = i_point * simToLocal();
        return true;
    }
    else
    {
        return false;
    }
}

//-*****************************************************************************
// Does the region intersect the point, with closest point info...
bool Mesh::intersects( const V3d& i_point,
                       BestMeshPointD& o_meshPoint ) const
{
    // First check bounds.
    if ( !simBounds().intersects( i_point ) ) { return false; }

    // Transform point internal
    V3f internalPoint = V3f( ( i_point * simToShape() ) );

    // If not inside, return false.
    if ( !m_triMesh->isInside( internalPoint ) ) { return false; }

    // Find the closest point.
    Etm::ClosestTriangle ctri =
        m_triMesh->closestPoint( internalPoint );
    if ( ctri.foundAny() )
    {
        // Barycentric is now correct.
        V2f baryAlphaBeta;
        float baryElevation;
        ctri.bestTriangle->computeBarycentric( internalPoint,
                                               baryAlphaBeta,
                                               baryElevation );
        internalPoint = ctri.bestTriangle->barycentricEval( baryAlphaBeta,
                                                            baryElevation );
        // CJH HACK
        #if 0
        o_meshPoint.simPoint = internalPoint * shapeToSim();
        #else
        o_meshPoint.simPoint = i_point;
        #endif
        o_meshPoint.shapePoint = internalPoint;
        o_meshPoint.barycentric = V3d( baryAlphaBeta.x,
                                       baryAlphaBeta.y,
                                       baryElevation );
        o_meshPoint.triId = ctri.bestTriangle->triangleId();
        o_meshPoint.meshId = m_meshId;
        const V3f svel = ctri.bestTriangle->velBarycentric(
                             baryAlphaBeta );
        V3f ovel;
        shapeToSim().multDirMatrix( svel, ovel );
        o_meshPoint.simVelocity = V3d( ovel );

        o_meshPoint.bestSquaredDist = ctri.bestSquaredDist;
        o_meshPoint.maxSquaredDist = ctri.maxSquaredDist;
        o_meshPoint.bestTriangle = ctri.bestTriangle;
        o_meshPoint.bestMesh = this;

        return true;
    }
    else
    {
        return false;
    }
}

//-*****************************************************************************
// Does a narrow interior band of the region intersect the bounds?
// This is an optimization only, and by default just returns intersects.
bool Mesh::intersectsInnerNarrowBand( const Box3d& i_bounds,
                                      double i_narrowBand2 ) const
{
    // This one's tough... for now just return intersects.
    return intersects( i_bounds );
}

//-*****************************************************************************
// Does a narrow interior band of the region intersect the point?
bool Mesh::intersectsInnerNarrowBand( const V3d& i_point,
                                      double i_narrowBand2,
                                      BestMeshPointD& o_meshPoint ) const
{
    // First check bounds.
    if ( !simBounds().intersects( i_point ) ) { return false; }

    // Transform point internal
    V3f internalPoint = V3f( ( i_point * simToShape() ) );

    // If not inside, return false.
    if ( !m_triMesh->isInside( internalPoint ) ) { return false; }

    Etm::ClosestTriangle ctri =
        m_triMesh->closestPointWithinSquaredDistance(
            internalPoint, i_narrowBand2 );
    if ( ctri.foundAny() )
    {
        // Barycentric is now correct.
        V2f baryAlphaBeta;
        float baryElevation;
        ctri.bestTriangle->computeBarycentric( internalPoint,
                                               baryAlphaBeta,
                                               baryElevation );
        internalPoint = ctri.bestTriangle->barycentricEval( baryAlphaBeta,
                                                            baryElevation );
        // CJH HACK
        #if 0
        o_meshPoint.simPoint = internalPoint * shapeToSim();
        #else
        o_meshPoint.simPoint = i_point;
        #endif
        o_meshPoint.shapePoint = internalPoint;
        o_meshPoint.barycentric = V3d( baryAlphaBeta.x,
                                       baryAlphaBeta.y,
                                       baryElevation );
        o_meshPoint.triId = ctri.bestTriangle->triangleId();
        o_meshPoint.meshId = m_meshId;
        const V3f svel = ctri.bestTriangle->velBarycentric(
                             baryAlphaBeta );
        V3f ovel;
        shapeToSim().multDirMatrix( svel, ovel );
        o_meshPoint.simVelocity = V3d( ovel );

        o_meshPoint.bestSquaredDist = ctri.bestSquaredDist;
        o_meshPoint.maxSquaredDist = ctri.maxSquaredDist;
        o_meshPoint.bestTriangle = ctri.bestTriangle;
        o_meshPoint.bestMesh = this;

        return true;
    }
    else
    {
        return false;
    }
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

