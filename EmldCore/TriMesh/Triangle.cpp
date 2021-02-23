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

#include "Triangle.h"
#include "Utility.h"

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
void Triangle::init()
{
    // Rotate Min To Zero. This guarantees precise ordering.
    rotateMinToZero( m_vertices );

    const Vertex *A = m_vertices[0];
    const Vertex *B = m_vertices[1];
    const Vertex *C = m_vertices[2];

    //-*************************************************************************
    // 2D STUFF
    // Make 2D points
    m_points2D[0] = V2f( A->position().y, A->position().z );
    m_points2D[1] = V2f( B->position().y, B->position().z );
    m_points2D[2] = V2f( C->position().y, C->position().z );

    // And depths
    m_depths2D[0] = A->position().x;
    m_depths2D[1] = B->position().x;
    m_depths2D[2] = C->position().x;

    // Bounds and center
    m_bounds2D.makeEmpty();
    m_bounds2D.extendBy( m_points2D[0] );
    m_bounds2D.extendBy( m_points2D[1] );
    m_bounds2D.extendBy( m_points2D[2] );

    m_center2D = ( m_points2D[0] + m_points2D[1] + m_points2D[2] ) / 3.0f;

    // Projection
    {
        float a = m_points2D[1].x - m_points2D[0].x;
        float b = m_points2D[2].x - m_points2D[0].x;
        float c = m_points2D[1].y - m_points2D[0].y;
        float d = m_points2D[2].y - m_points2D[0].y;
        float det = ( a * d ) - ( b * c );
        if ( std::abs( det ) < 1.0e-6f )
        {
            // Projection cannot be inverted, so make it always degenerate.
            m_projection[0][0] = 0.0f;
            m_projection[0][1] = 0.0f;
            m_projection[1][0] = 0.0f;
            m_projection[1][1] = 0.0f;
        }
        else
        {
            // Invert the matrix
            m_projection[0][0] = d / det;
            m_projection[0][1] = -b / det;
            m_projection[1][0] = -c / det;
            m_projection[1][1] = a / det;
        }
    }

    //-*************************************************************************
    // 3D STUFF

    // Edges, precisely ordered.
    m_edgeOrigins[0] = m_vertices[0]->position();
    m_edgeOrigins[1] = m_vertices[0]->position(); // Not a typo.

    m_edgeUnitDirs[0] = m_vertices[1]->position() - m_edgeOrigins[0];
    m_edgeUnitDirs[1] = m_vertices[2]->position() - m_edgeOrigins[1];

    m_edgeLengths[0] = m_edgeUnitDirs[0].length();
    m_edgeLengths[1] = m_edgeUnitDirs[1].length();

    m_edgeUnitDirs[0].normalize();
    m_edgeUnitDirs[1].normalize();

    // Compute planeN and planeD
    // Again, with precise ordering.
    if ( m_vertices[1] < m_vertices[2] )
    {
        m_planeN = m_edgeUnitDirs[0].cross( m_edgeUnitDirs[1] );
        m_planeN.normalize();

        m_edgeOrigins[2] = m_vertices[1]->position();
        m_edgeUnitDirs[2] = m_vertices[2]->position() - m_edgeOrigins[2];

        m_edgeLengths[2] = m_edgeUnitDirs[2].length();
        m_edgeUnitDirs[2].normalize();
    }
    else
    {
        m_planeN = m_edgeUnitDirs[1].cross( m_edgeUnitDirs[0] );
        m_planeN.normalize();
        m_planeN = -m_planeN;

        m_edgeOrigins[2] = m_vertices[2]->position();
        m_edgeUnitDirs[2] = m_vertices[1]->position() - m_edgeOrigins[2];
        m_edgeLengths[2] = m_edgeUnitDirs[2].length();
        m_edgeUnitDirs[2].normalize();
    }

    // Plane equation is ( m_planeN . somePointOnPlane ) + D = 0
    // D = -( m_planeN . somePointOnPlane )
    m_planeD = -( m_planeN.dot( m_vertices[0]->position() ) );

    // Projection axis is the one with the largest dot product
    // against planeN.
    // We need this for closest point tests.
    if ( std::abs( m_planeN.x ) > std::abs( m_planeN.y ) )
    {
        if ( std::abs( m_planeN.x ) > std::abs( m_planeN.z ) )
        {
            m_projectionAxis = 0;
        }
        else
        {
            m_projectionAxis = 2;
        }
    }
    else
    {
        if ( std::abs( m_planeN.y ) > std::abs( m_planeN.z ) )
        {
            m_projectionAxis = 1;
        }
        else
        {
            m_projectionAxis = 2;
        }
    }

    // Projected points
    // Now project into 2D.
    switch ( m_projectionAxis )
    {
    case 0:
        m_p2[0] = V2f( m_vertices[0]->position().y, m_vertices[0]->position().z );
        m_p2[1] = V2f( m_vertices[1]->position().y, m_vertices[1]->position().z );
        m_p2[2] = V2f( m_vertices[2]->position().y, m_vertices[2]->position().z );
        break;
    case 1:
        m_p2[0] = V2f( m_vertices[0]->position().x, m_vertices[0]->position().z );
        m_p2[1] = V2f( m_vertices[1]->position().x, m_vertices[1]->position().z );
        m_p2[2] = V2f( m_vertices[2]->position().x, m_vertices[2]->position().z );
        break;
    case 2:
        m_p2[0] = V2f( m_vertices[0]->position().x, m_vertices[0]->position().y );
        m_p2[1] = V2f( m_vertices[1]->position().x, m_vertices[1]->position().y );
        m_p2[2] = V2f( m_vertices[2]->position().x, m_vertices[2]->position().y );
        break;
    };

    // Barycentric Matrix.
    m_barycentricMatrix = M33f( m_p2[0].x, m_p2[0].y, 1.0f,
                                m_p2[1].x, m_p2[1].y, 1.0f,
                                m_p2[2].x, m_p2[2].y, 1.0f );
    m_barycentricMatrix.gjInvert( false );

    // Bounds and center
    m_bounds.makeEmpty();
    m_bounds.extendBy( m_vertices[0]->position() );
    m_bounds.extendBy( m_vertices[1]->position() );
    m_bounds.extendBy( m_vertices[2]->position() );
    m_center = ( m_vertices[0]->position() +
                 m_vertices[1]->position() +
                 m_vertices[2]->position() ) / 3.0f;

    V3f AB = m_vertices[1]->position() - m_vertices[0]->position();
    V3f AC = m_vertices[2]->position() - m_vertices[0]->position();
    m_area = 0.5f * ( AB.cross( AC ) ).length();
}

//-*****************************************************************************
// does a line cast from point P in the X direction hit the line segment
// defined by A-B? This function is exact - will always return the same
// answer for the same inputs in any order.
static bool __testPointVsSegmentNoSwap( const V2f &A,
                                        const V2f &B,
                                        const V2f &P )
{
    bool say = A.y >= P.y;
    bool sby = B.y >= P.y;

    if ( say == sby )
    {
        return false;
    }
    else
    {
        bool sax = A.x > P.x;
        bool sbx = B.x > P.x;
        if ( sax && sbx )
        {
            return true;
        }
        else if ( sax || sbx )
        {
            float xint = A.x + ( ( B.x - A.x ) * ( P.y - A.y ) / ( B.y - A.y ) );
            return xint >= P.x;
        }
        else
        {
            return false;
        }
    }

    return false;
}

//-*****************************************************************************
static inline bool testPointVsSegment( const V2f &a,
                                       const V2f &b,
                                       const V2f &p )
{
    if ( b.y < a.y || ( b.y == a.y && b.x < a.x ) )
    {
        return __testPointVsSegmentNoSwap( b, a, p );
    }
    else
    {
        return __testPointVsSegmentNoSwap( a, b, p );
    }
}

//-*****************************************************************************
// This tests whether or not a ray from the 2d point P in the plane, in a
// direction perpendicular to the plane and starting at a height "d"
// above the plane and traveling up to infinity,
// intersects a triangle of three vertices with three depths in the plane.
//
// So, first it checks to see whether d is higher than all the depths
// in which case it could never hit.
// Then it checks to see whether or not the hit is in the triangle in the
// plane. If it is, then depth ranges are compared to avoid a barycentric
// test if possible. If necessary, finally the exact depth of the intersection
// of the ray with the triangle is found, and that depth is compared to d.
bool Triangle::hit2D( const V2f &p, float d ) const
{
    if ( d > m_depths2D[0] && d > m_depths2D[1] && d > m_depths2D[2] )
    {
        return false;
    }

    if ( m_bounds2D.intersects( p ) == false )
    {
        return false;
    }

    // Find out if a hit happened in the plane.
    int NC = 0;
    NC += ( int )( testPointVsSegment( m_points2D[0], m_points2D[1], p ) );
    NC += ( int )( testPointVsSegment( m_points2D[1], m_points2D[2], p ) );
    if ( NC < 2 )
    {
        NC += ( int )( testPointVsSegment( m_points2D[2], m_points2D[0], p ) );
    }

    if ( NC != 1 )
    {
        return false;
    }

    // Okay, a hit did happen in the plane! Hopefully we don't need
    // a barycentric test.
    if ( d <= m_depths2D[0] && d <= m_depths2D[1] && d <= m_depths2D[2] )
    {
        // Since the starting depth of the ray is less than the depth
        // of all the vertices, a hit will definitely happen.
        return true;
    }
    else
    {
        // Damn - barycentric test needed.
        float sx = p.x - m_points2D[0].x;
        float sy = p.y - m_points2D[0].y;
        float s = ( m_projection[0][0] * sx ) + ( m_projection[0][1] * sy );
        float t = ( m_projection[1][0] * sx ) + ( m_projection[1][1] * sy );
        s = Imath::clamp( s, 0.0f, 1.0f );
        t = Imath::clamp( t, 0.0f, 1.0f - s );

        float du = m_depths2D[1] - m_depths2D[0];
        float dv = m_depths2D[2] - m_depths2D[0];

        float depth = m_depths2D[0] + ( s * du ) + ( t * dv );

        return ( d <= depth );
    }
}

//******************************************************************************
bool Triangle::hitDepth2D( const V2f &p, float &hitDepth ) const
{
    if ( m_bounds2D.intersects( p ) == false )
    {
        return false;
    }

    // Find out if a hit happened in the plane.
    int NC = 0;
    NC += ( int )( testPointVsSegment( m_points2D[0], m_points2D[1], p ) );
    NC += ( int )( testPointVsSegment( m_points2D[1], m_points2D[2], p ) );
    if ( NC < 2 )
    {
        NC += ( int )( testPointVsSegment( m_points2D[2], m_points2D[0], p ) );
    }

    if ( NC != 1 )
    {
        return false;
    }

    // Damn - barycentric test needed.
    float sx = p.x - m_points2D[0].x;
    float sy = p.y - m_points2D[0].y;
    float s = ( m_projection[0][0] * sx ) + ( m_projection[0][1] * sy );
    float t = ( m_projection[1][0] * sx ) + ( m_projection[1][1] * sy );
    s = Imath::clamp( s, 0.0f, 1.0f );
    t = Imath::clamp( t, 0.0f, 1.0f - s );

    float du = m_depths2D[1] - m_depths2D[0];
    float dv = m_depths2D[2] - m_depths2D[0];

    hitDepth = m_depths2D[0] + ( s * du ) + ( t * dv );

    return true;
}

//-*****************************************************************************
//-*****************************************************************************
// 3D STUFF
//-*****************************************************************************
//-*****************************************************************************
bool Triangle::intersectSegment( const V3f &origin,
                                 const V3f &destination,
                                 V3f &pFound,
                                 float tol ) const
{
    // Get bounds of segment
    Box3f sBnds;
    sBnds.min.x = std::min( origin.x, destination.x );
    sBnds.min.y = std::min( origin.y, destination.y );
    sBnds.min.z = std::min( origin.z, destination.z );

    sBnds.max.x = std::max( origin.x, destination.x );
    sBnds.max.y = std::max( origin.y, destination.y );
    sBnds.max.z = std::max( origin.z, destination.z );

    if ( !sBnds.intersects( m_bounds ) )
    {
        return false;
    }

    // First, figure out if the segment even intersects the plane
    // and if so, where.
    V3f dir = destination - origin;
    float len = dir.length();
    if ( len < 1.0e-4f )
    {
        // Segment too short to intersect anything
        return false;
    }
    V3f dirN = dir / len;

    float vd = dirN.dot( m_planeN );
    if ( std::abs( vd ) < 1.0e-4f )
    {
        // Segment is nearly parallel to plane of triangle.
        // We'll call that a non-hit.
        return false;
    }

    float v0 = -( origin.dot( m_planeN ) + m_planeD );

    float t = v0 / vd;
    if ( t < 0.0f || t > len )
    {
        // Segment intersects plane behind start point or after end point,
        return false;
    }

    // Calculate intersection point.
    V3f p = origin + ( t * dirN );

    // Now project into 2D.
    V2f p2;
    switch ( m_projectionAxis )
    {
    case 0:
        p2 = V2f( p.y, p.z );
        break;
    case 1:
        p2 = V2f( p.x, p.z );
        break;
    case 2:
        p2 = V2f( p.x, p.y );
        break;
    };

    // Now check hit list
    int NC = 0;
    NC += ( int )testPointVsSegment( m_p2[0], m_p2[1], p2 );
    NC += ( int )testPointVsSegment( m_p2[1], m_p2[2], p2 );
    if ( NC < 2 )
    {
        NC += ( int )testPointVsSegment( m_p2[0], m_p2[2], p2 );
    }

    // If the edge intersected, calculate the end point.
    if ( NC == 1 )
    {
        // std::cout << "t = " << t << ", len = " << len
        //          << std::endl;
        pFound = p;
        return true;
    }

    return false;
}

//-*****************************************************************************
V3f Triangle::closestPoint( const V3f &p, float &retD2 ) const
{
    const V3f &v0 = m_vertices[0]->position();
    const V3f &v1 = m_vertices[1]->position();
    const V3f &v2 = m_vertices[2]->position();

    V3f dp = p - v0;
    float proj = dp.dot( m_planeN );
    V3f projP = p - proj * m_planeN;

    // Now project into 2D.
    V2f p2;
    switch ( m_projectionAxis )
    {
    case 0:
        p2 = V2f( projP.y, projP.z );
        break;
    case 1:
        p2 = V2f( projP.x, projP.z );
        break;
    case 2:
        p2 = V2f( projP.x, projP.y );
        break;
    };

    // Now check hit list
    int NC = 0;
    NC += ( int )testPointVsSegment( m_p2[0], m_p2[1], p2 );
    NC += ( int )testPointVsSegment( m_p2[1], m_p2[2], p2 );
    if ( NC < 2 )
    {
        NC += ( int )testPointVsSegment( m_p2[0], m_p2[2], p2 );
    }

    if ( NC == 1 )
    {
        // The point on the plane is the one.
        retD2 = proj * proj;
        return projP;
    }
    else
    {
        // We need to test the three corners AND
        // the lines between them.
        // NOT JUST THE CORNERS! THIS CODE HAS BEEN WRONG
        // FOR TEN YEARS!!! ARRRRGGGHGHHHH!

        // Check edges first. Only if none of the edges
        // hit do we bother with corners

        // Use first point, since we have it alreayd
        float bestD2 = dp.dot( dp );
        V3f bestPoint = v0;
        bool foundEdge = false;

        for ( int edge = 0; edge < 3; ++edge )
        {
            V3f dEdge = p - m_edgeOrigins[edge];
            float edgeProj = dEdge.dot( m_edgeUnitDirs[edge] );

            if ( edgeProj >= 0.0f &&
                 edgeProj <= m_edgeLengths[edge] )
            {
                V3f edgePoint = m_edgeOrigins[edge] +
                    edgeProj * m_edgeUnitDirs[edge];

                float edgeD2 = ( p - edgePoint ).length2();

                if ( edgeD2 < bestD2 )
                {
                    bestD2 = edgeD2;
                    bestPoint = edgePoint;
                    foundEdge = true;
                }
            }
        }

        if ( !foundEdge )
        {
            // Check Second corner.
            V3f dp1 = p - v1;
            float d2V1 = dp1.dot( dp1 );
            if ( d2V1 < bestD2 )
            {
                bestD2 = d2V1;
                bestPoint = v1;
            }

            // Check third corner.
            V3f dp2 = p - v2;
            float d2V2 = dp2.dot( dp2 );
            if ( d2V2 < bestD2 )
            {
                bestD2 = d2V2;
                bestPoint = v2;
            }
        }

        // Aggressive debug.
#if 0
        if ( !SpatialSubd::PointBoxMinimumSquaredDistanceLessThan
             ( p, m_bounds, 1.01f * bestD2 ) )
        {
            Imath::Intervalf ivl =
                SpatialSubd::PointBoxSquaredDistanceInclusion( p,
                                                               m_bounds );
            std::cerr << "Bad bad things." << std::endl
                      << "Interval: " << ivl.min << " to "
                      << ivl.max << std::endl
                      << "Best D2: " << bestD2 << std::endl;
            abort();
        }
#endif

        retD2 = bestD2;
        return bestPoint;
    }
}

//-*****************************************************************************
float Triangle::squaredDistanceToPoint( const V3f &p ) const
{
#if 0
    V3f dp = p - m_vertices[0]->position();
    float proj = dp.dot( m_planeN );
    V3f projP = p - proj * m_planeN;

    // Now project into 2D.
    V2f p2;
    switch ( m_projectionAxis )
    {
    case 0:
        p2 = V2f( projP.y, projP.z );
        break;
    case 1:
        p2 = V2f( projP.x, projP.z );
        break;
    case 2:
        p2 = V2f( projP.x, projP.y );
        break;
    };

    // Now check hit list
    int NC = 0;
    NC += ( int )testPointVsSegment( m_p2[0], m_p2[1], p2 );
    NC += ( int )testPointVsSegment( m_p2[1], m_p2[2], p2 );
    if ( NC < 2 )
    {
        NC += ( int )testPointVsSegment( m_p2[0], m_p2[2], p2 );
    }

    if ( NC == 1 )
    {
        return proj * proj;
    }
    else
    {
        float dMin = dp.dot( dp );
        dp = V3f( p - m_vertices[1]->position() );
        dMin = std::min( dMin, dp.dot( dp ) );
        dp = V3f( p - m_vertices[2]->position() );
        return std::min( dMin, dp.dot( dp ) );
    }
#else
    float d2 = 0.0f;
    closestPoint( p, d2 );
    return d2;
#endif
}

//-*****************************************************************************
void Triangle::computeBarycentric( const V3f& i_p,
                                   V2f& o_baryAlphaBeta,
                                   float& o_elevation ) const
{
    V3f dp = i_p - m_vertices[0]->position();
    o_elevation = dp.dot( m_planeN );
    V3f planePos = m_vertices[0]->position() +
        ( dp - ( o_elevation * m_planeN ) );

    // Now project into 2D.
    V3f p2;
    switch ( m_projectionAxis )
    {
    case 0:
        p2 = V3f( planePos.y, planePos.z, 1.0f );
        break;
    case 1:
        p2 = V3f( planePos.x, planePos.z, 1.0f );
        break;
    case 2:
        p2 = V3f( planePos.x, planePos.y, 1.0f );
        break;
    };

    V3f alphaBetaGamma = p2 * m_barycentricMatrix;
    o_baryAlphaBeta.x = alphaBetaGamma.x;
    o_baryAlphaBeta.y = alphaBetaGamma.y;
}

//-*****************************************************************************
// Not exactly accurate, but close enough.
V3f Triangle::velAtClosestPoint( const V3f &p ) const
{
    // Slow, but works for now - fix later.
    float d2_0 = ( p - m_vertices[0]->position() ).length2();
    float d2_1 = ( p - m_vertices[1]->position() ).length2();
    float d2_2 = ( p - m_vertices[2]->position() ).length2();

    float w0 = ( d2_1 + d2_2 );
    float w1 = ( d2_0 + d2_2 );
    float w2 = ( d2_0 + d2_1 );

    return ( ( w0*m_vertices[0]->velocity() ) +
             ( w1*m_vertices[1]->velocity() ) +
             ( w2*m_vertices[2]->velocity() ) ) /
        std::max( 0.0001f, ( w0 + w1 + w2 ) );
}

} // End namespace TriMesh
} // End namespace EmldCore


