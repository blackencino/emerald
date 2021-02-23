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

#ifndef _EmldCore_ElutMesh_Cell_h_
#define _EmldCore_ElutMesh_Cell_h_

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
#include "PreVertex.h"
#include "PreTriangle.h"
#include "LUT.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
// What does a Cell do? A cell is an 8-cornered rectangular cube in some
// coordinate space which has level set values on each corner.  If the cube
// contains a surface crossing - by which we mean, if the cube has some corners
// greater than or equal to zero and others less than zero, an index can be
// produced which allows for a lookup into the ExhaustiveLUT, which provides
// triangle descriptions relative to the zero-crossings on the edges and
// interior diagonals of the cube.
//
// When the cell produces a mesh, it does so by creating vertices on edges
// and diagonals. Since the edges and diagonals can each only have a single
// zero-crossing, the vertex associated with any corner coordinate pair is
// unique and can be referenced by the integer coordinates of the two vertices.
//
// This class assumes the existence of a mesh class, which can create and
// return vertices as well as triangles.
// The problem is that if we're processing cells in parallel, we produce
// vertices which are duplicates across different threads, and then we have
// triangles that point to vertices that will change, once the duplicates
// are merged.
//
// Solution:
// Make a "Pre Vertex" class and a "Pre Triangle" class. The Pre Vertex class
// is just the vertex's unique name, two V3i's, but it also stores
// the resolved position and filter axes. The Pre Triangle class refers
// to the Pre Vertices by name, explicitly.  We accumulate the pv's and pt's in
// BCV's. Yay for acronyms.  We then run the usual process of parallel
// de-duplication - sort, prefix sums at value changes, and creation of
// blocks with begins & ends.  The cool part is that we can treat each
// triangle that has a vertex...
//-*****************************************************************************

template <typename MBUILD>
class Cell
{
public:
    typedef typename MBUILD::value_type T;
    typedef typename Imath::Vec3<T> V3T;

    // Make a cell. It will check the level sets, and if
    // there's a zero crossing, emit pre-verts and pre-tris.
    Cell( MBUILD& o_meshBuilder,
          std::size_t i_jStride,
          std::size_t i_kStride,
          const V3i& i_origin,
          const T* i_stridedLevelSets,
          const V3T* i_stridedPs,
          const V3T* i_stridedDpDus,
          const V3T* i_stridedDpDvs,
          const V3T* i_stridedDpDws );

    void computeSaddlePoints( SaddlePointsT<T>& o_sp ) const;

protected:
    void getEdge( int i_edge, PreVertex<T>& o_vertex ) const;
    void getDiagonal( int i_diag, PreVertex<T>& o_vertex ) const;
    void getVertex( int i_edgeNumber, PreVertex<T>& o_vertex ) const;

    MBUILD& m_meshBuilder;
    std::size_t m_jStride;
    std::size_t m_kStride;
    V3i m_origin;
    const T* m_stridedLevelSets;
    const V3T* m_stridedPs;
    const V3T* m_stridedDpDus;
    const V3T* m_stridedDpDvs;
    const V3T* m_stridedDpDws;

    // We re-bake into the locations we need, switching signs along the way.
    V3i m_corners[8];
    T m_levelSet[8];
    V3T m_p[8];
    V3T m_dPdu[8];
    V3T m_dPdv[8];
    V3T m_dPdw[8];
};

//-*****************************************************************************
// CJH HACK - SO, SO, MUCH OF THIS COULD BE OPTIMIZED.
// It takes a large amount of work right now to determine that
// the cell contains no zero-crossings.
template <typename MBUILD>
Cell<MBUILD>::Cell
(
    MBUILD& o_meshBuilder,
    std::size_t i_jStride,
    std::size_t i_kStride,
    const V3i& i_origin,
    const T* i_stridedLevelSets,
    const V3T* i_stridedPs,
    const V3T* i_stridedDpDus,
    const V3T* i_stridedDpDvs,
    const V3T* i_stridedDpDws
)
    : m_meshBuilder( o_meshBuilder )
    , m_jStride( i_jStride )
    , m_kStride( i_kStride )
    , m_origin( i_origin )
    , m_stridedLevelSets( i_stridedLevelSets )
    , m_stridedPs( i_stridedPs )
    , m_stridedDpDus( i_stridedDpDus )
    , m_stridedDpDvs( i_stridedDpDvs )
    , m_stridedDpDws( i_stridedDpDws )
{
    // Set the 8 corners. This iterates against their logical
    // order.
    m_corners[kNegxNegyNegz] = m_origin + V3i( 0, 0, 0 );
    m_corners[kPosxNegyNegz] = m_origin + V3i( 1, 0, 0 );
    m_corners[kNegxPosyNegz] = m_origin + V3i( 0, 1, 0 );
    m_corners[kPosxPosyNegz] = m_origin + V3i( 1, 1, 0 );
    m_corners[kNegxNegyPosz] = m_origin + V3i( 0, 0, 1 );
    m_corners[kPosxNegyPosz] = m_origin + V3i( 1, 0, 1 );
    m_corners[kNegxPosyPosz] = m_origin + V3i( 0, 1, 1 );
    m_corners[kPosxPosyPosz] = m_origin + V3i( 1, 1, 1 );

    // Set the 8 level sets
    const V3i stride( 1, m_jStride, m_kStride );
    for ( int i = 0; i < 8; ++i )
    {
        // Our level sets have negative inside, but the ELUT logic
        // assumes positive inside.
        int idx = ( m_corners[i] - m_origin ).dot( stride );
        EMLD_ASSERT( idx >= 0 && idx <= ( 1 + m_jStride + m_kStride ),
                     "Bad index: " << idx );
        m_levelSet[i] = -m_stridedLevelSets[idx];
        EMLD_ASSERT( std::isfinite( m_stridedLevelSets[idx] ) &&
                           std::isfinite( m_levelSet[i] ),
                           "NAN level set!" );
        EMLD_ASSERT( QNEGATIVE( m_levelSet[i] ) !=
                           QPOSITIVE( m_levelSet[i] ),
                           "NAN level set 2!" );

        m_p[i] = m_stridedPs[idx];
        m_dPdu[i] = m_stridedDpDus[idx];
        m_dPdv[i] = m_stridedDpDvs[idx];
        m_dPdw[i] = m_stridedDpDws[idx];
    }

    // Figure out the index into the exhaustive lut.
    // The ordering of corners here is odd, and corresponds to the
    // ordering described in the paper.
    int index = 0;
    if ( QNEGATIVE( m_levelSet[0] ) )   { index |= 0x1<<0; }
    if ( QNEGATIVE( m_levelSet[1] ) )   { index |= 0x1<<1; }
    if ( QNEGATIVE( m_levelSet[2] ) )   { index |= 0x1<<2; }
    if ( QNEGATIVE( m_levelSet[3] ) )   { index |= 0x1<<3; }
    if ( QNEGATIVE( m_levelSet[4] ) )   { index |= 0x1<<4; }
    if ( QNEGATIVE( m_levelSet[5] ) )   { index |= 0x1<<5; }
    if ( QNEGATIVE( m_levelSet[6] ) )   { index |= 0x1<<6; }
    if ( QNEGATIVE( m_levelSet[7] ) )   { index |= 0x1<<7; }

    // Test to see whether it's all negative or all positive.
    EMLD_LITERAL_CONSTANT int allNeg =
        ( ( 0x1<<0 ) |
          ( 0x1<<1 ) |
          ( 0x1<<2 ) |
          ( 0x1<<3 ) |
          ( 0x1<<4 ) |
          ( 0x1<<5 ) |
          ( 0x1<<6 ) |
          ( 0x1<<7 ) );
    if ( index == 0 || index == allNeg )
    {
        // No triangles
        return;
    }

    // Indexing scheme is [index][variation][triangle]
    const Variations& vars = g_LUT[ index ];
    if ( vars.size() <= 0 )
    {
        // This means no triangles are created.
        return;
    }

    // Make three verts for triangles.
    PreVertex<T> v0;
    PreVertex<T> v1;
    PreVertex<T> v2;

    // Only one variation for this configuration. Make its triangles
    // and be done!
    if ( vars.size() == 1 )
    {
        // No need for saddle checking.
        const Variation& var = vars[0];
        int numTris = var.size();
        for ( int tri = 0; tri < numTris; ++tri )
        {
            this->getVertex( var[tri][0], v0 );
            this->getVertex( var[tri][1], v1 );
            this->getVertex( var[tri][2], v2 );
            m_meshBuilder.emitPreTriangle( v0, v1, v2 );
        }

        // Single variation's triangles have been made, we're done!
        return;
    }

    // If we get here, the variation has size > 1.
    EMLD_DEBUG_ASSERT( vars.size() > 1, "Should have more than one variation" );

    // Make saddle points. These help us pick a variation.
    SaddlePointsT<T> sp;
    computeSaddlePoints( sp );

    // Find our variation, make tris from it.
    for ( int i = 0; i < vars.size(); ++i )
    {
        const Variation& var = vars[i];
        if ( var.match( sp ) )
        {
            int numTris = var.size();
            for ( int tri = 0; tri < numTris; ++tri )
            {
                this->getVertex( var[tri][0], v0 );
                this->getVertex( var[tri][1], v1 );
                this->getVertex( var[tri][2], v2 );
                m_meshBuilder.emitPreTriangle( v0, v1, v2 );
            }
        }
    }
}

//-*****************************************************************************
// This saddle point computation, which computes the six face saddle points
// and then a body saddle point as the seventh, is the key used to
// disambiguate different triangle configurations, and is the main
// computation in the paper. The formulation here is copied more or less
// verbatim.
template <typename MBUILD>
void Cell<MBUILD>::computeSaddlePoints( SaddlePointsT<T>& o_sp ) const
{
    // These again are in weird order, the order expected by the paper.
    T v0 = m_levelSet[0];
    T v1 = m_levelSet[1];
    T v2 = m_levelSet[2];
    T v3 = m_levelSet[3];
    T v4 = m_levelSet[4];
    T v5 = m_levelSet[5];
    T v6 = m_levelSet[6];
    T v7 = m_levelSet[7];

    T a = v1 + v3 + v4 + v6 - v0 - v7 - v5 - v2;
    T b = v0 + v2 - v1 - v3;
    T c = v0 + v7 - v4 - v3;
    T d = v0 + v5 - v1 - v4;
    T e = v1 - v0;
    T f = v3 - v0;
    T g = v4 - v0;
    T h = v0;

    o_sp.data[0] = h - ( ( f * g ) / c );
    o_sp.data[1] = ( ( v1 * v6 ) - ( v2 * v5 ) ) / ( v1 + v6 - v2 - v5 );
    o_sp.data[2] = h - ( ( e * g ) / d );
    o_sp.data[3] = ( ( v3 * v6 ) - ( v2 * v7 ) ) / ( v3 + v6 - v2 - v7 );
    o_sp.data[4] = h - ( ( e * f ) / b );
    o_sp.data[5] = ( ( v4 * v6 ) - ( v5 * v7 ) ) / ( v4 + v6 - v5 - v7 );

    T R = ( ( b * d ) - ( a * e ) ) * ( ( b * c ) - ( a * f ) ) *
    ( ( c * d ) - ( a * g ) );
    if ( R < T( 0 ) )
    {
        o_sp.data[6] = T( 0 );
        return;
        // EMLD_THROW( "ElutMesh::Cell: Negative Discriminant" );
    }
    R = std::sqrt( R );

    T denomX = a * ( ( a * e ) - ( b * d ) );
    T denomY = a * ( ( a * f ) - ( b * c ) );
    T denomZ = a * ( ( a * g ) - ( c * d ) );

    T numerX = ( b * c * d ) - ( a * c * e );
    T numerY = ( b * c * d ) - ( a * d * f );
    T numerZ = ( b * c * d ) - ( a * b * g );

    // CJH HACK: This is not a great way to check for valid divisors.
    if ( std::abs( denomX ) < std::numeric_limits<T>::epsilon() ||
         std::abs( denomY ) < std::numeric_limits<T>::epsilon() ||
         std::abs( denomZ ) < std::numeric_limits<T>::epsilon() )
    {
        o_sp.data[6] = T( 0 );
        return;
        // EMLD_THROW( "ElutMesh::Cell: Undefined Saddle Points 1" );
    }

    V3T Sp0( ( numerX + R ) / denomX,
             ( numerY + R ) / denomY,
             ( numerZ + R ) / denomZ );

    V3T Sp1( ( numerX - R ) / denomX,
             ( numerY - R ) / denomY,
             ( numerZ - R ) / denomZ );

    if ( ( a * a ) < std::numeric_limits<T>::epsilon() )
    {
        o_sp.data[6] = T( 0 );
        return;
        // EMLD_THROW( "ElutMesh::Cell: Undefined Saddle Points 2" );
    }

    T Sv0 =
        ( ( a * a * h ) - ( a * b * g ) - ( a * d * f ) - ( a * c * e ) +
          ( T( 2 ) * b * c * d ) + ( T( 2 ) * R * R ) ) /
        ( a * a );

    T Sv1 =
        ( ( a * a * h ) - ( a * b * g ) - ( a * d * f ) - ( a * c * e ) +
          ( T( 2 ) * b * c * d ) - ( T( 2 ) * R * R ) ) /
        ( a * a );

    //static const B3T unitCube( V3T( T(0) ), V3T( T(1) ) );

    T r20 = ( Sp0 - V3T( T( 0.5 ) ) ).length2();
    T r21 = ( Sp1 - V3T( T( 0.5 ) ) ).length2();

    if ( r20 < r21 )
    {
        o_sp.data[6] = Sv0;
    }
    else
    {
        //if ( !unitCube.intersects( Sp1 ) )
        //{
        //EMLD_THROW( "ElutMesh::Cell: Undefined Saddle Points 3" );
        //}
        o_sp.data[6] = Sv1;
    }
}

//-*****************************************************************************
template <typename T>
bool CrossesSurface( T i_qA, T i_qB )
{
    return ( QNEGATIVE( i_qA ) != QNEGATIVE( i_qB ) );
}

//-*****************************************************************************
template <typename T>
Imath::Vec3<T>
FindZero( const Imath::Vec3<T>& i_pA, const Imath::Vec3<T>& i_pB,
          T i_qA, T i_qB )
{
    // Get interpolant. This solves for the zero crossing between qA & qB,
    // as a 0-1 from A to B.
    // note, if qA is negative, then qB is positive, so -qB is negative,
    // so qA - qB is negative, so then qA / ( qA - qB ) is positive and
    // between zero and 1.
    // If the signs are reversed, same thing.
    T interpolant = i_qA / ( i_qA - i_qB );
    if ( !std::isfinite( interpolant ) || interpolant <= T(0) )
    {
        return i_pA;
    }
    else if ( interpolant >= T(1) )
    {
        return i_pB;
    }
    else
    {
        return mix( i_pA, i_pB, interpolant );
    }
}

//-*****************************************************************************
template <typename T>
T FindZeroInterpolant( T i_qA, T i_qB )
{
    // Get interpolant. This solves for the zero crossing between qA & qB,
    // as a 0-1 from A to B.
    // note, if qA is negative, then qB is positive, so -qB is negative,
    // so qA - qB is negative, so then qA / ( qA - qB ) is positive and
    // between zero and 1.
    // If the signs are reversed, same thing.
    T interpolant = i_qA / ( i_qA - i_qB );
    if ( !std::isfinite( interpolant ) || interpolant <= T(0) )
    {
        return T(0);
    }
    else if ( interpolant >= T(1) )
    {
        return T(1);
    }
    else
    {
        return interpolant;
    }
}

//-*****************************************************************************
// This collection of magic numbers is created by carefully labeling
// the vertices and edges of the cell diagram from the paper,
// and collecting the associations.
template <typename MBUILD>
void Cell<MBUILD>::getEdge( int i_edge, PreVertex<T>& o_vertex ) const
{
    int main;
    int other;

    // This corresponds to the edge numbering as described in the paper.
    switch ( i_edge )
    {
        ////////
    case 1:
    case 13:
        main    = 0;
        other   = 1;
        break;
    case 2:
    case 14:
        main    = 1;
        other   = 2;
        break;
    case 3:
    case 15:
        main    = 2;
        other   = 3;
        break;
    case 4:
    case 16:
        main    = 0;
        other   = 3;
        break;

        /////////
    case 5:
    case 17:
        main    = 4;
        other   = 5;
        break;
    case 6:
    case 18:
        main    = 5;
        other   = 6;
        break;
    case 7:
    case 19:
        main    = 6;
        other   = 7;
        break;
    case 8:
    case 20:
        main    = 4;
        other   = 7;
        break;

        /////////
    case 9:
    case 21:
        main    = 0;
        other   = 4;
        break;

    case 10:
    case 22:
        main    = 1;
        other   = 5;
        break;

        ////////////
    case 11:
    case 23:
        main    = 3;
        other   = 7;
        break;

    case 12:
    case 24:
        main    = 2;
        other   = 6;
        break;

    default:
        EMLD_THROW( "ElutMesh::Cell: Invalid Edge: " << i_edge );
        break;
    };

    // Get the corners & level sets, check goodness.
    V3i cA = m_corners[main];
    V3i cB = m_corners[other];
    T qA = m_levelSet[main];
    T qB = m_levelSet[other];
    EMLD_DEBUG_ASSERT( cA != cB, "Degenerate edge" );
    EMLD_ASSERT( CrossesSurface( qA, qB ), "Edge must cross surface." );

    // Make sure they're ordered so that the edge vertex will be created
    // the same way each time.
    if ( (( const Imath::Vec3<int>& )cA) <
         (( const Imath::Vec3<int>& )cB) )
    {
        std::swap( cA, cB );
        std::swap( qA, qB );
        std::swap( main, other );
    }

    // Find the zero between them, set the vertex.
    T t = FindZeroInterpolant( qA, qB );

    o_vertex.set( cA, cB,
                  mix( m_p[main], m_p[other], t ),
                  mix( m_dPdu[main], m_dPdu[other], t ),
                  mix( m_dPdv[main], m_dPdv[other], t ),
                  mix( m_dPdw[main], m_dPdw[other], t ) );
}

//-*****************************************************************************
// This collection of magic numbers is created by carefully labeling
// the vertices and edges of the cell diagram from the paper,
// and collecting the associations.
// With the diagonals, the first "other" is the index of the other end of
// the diagonal. The three remaining others are, in no particular order,
// the indices of the corners that are connected directly or the origin corner.
// In a cube, each corner is directly connected to three other corners, and
// these are their indices.
// We gather other points, because along the diagonal there is not necessarily
// a guarantee of zero crossing, so we use averages of each zero crossing which
// might exist.
template <typename MBUILD>
void Cell<MBUILD>::getDiagonal( int i_diag, PreVertex<T>& o_vertex ) const
{
    int main;
    int other1;
    int other2;
    int other3;
    int other4;

    switch ( i_diag )
    {
    case 25:
        main   = 0;
        other1 = 6;

        other2 = 1;
        other3 = 3;
        other4 = 4;
        break;

    case 26:
        main   = 1;
        other1 = 7;

        other2 = 0;
        other3 = 2;
        other4 = 5;
        break;

    case 27:
        main   = 2;
        other1 = 4;

        other2 = 1;
        other3 = 3;
        other4 = 6;
        break;

    case 28:
        main   = 3;
        other1 = 5;

        other2 = 0;
        other3 = 2;
        other4 = 7;
        break;

    //////
    case 29:
        main   = 4;
        other1 = 2;

        other2 = 0;
        other3 = 5;
        other4 = 7;
        break;

    case 30:
        main   = 5;
        other1 = 3;

        other2 = 1;
        other3 = 4;
        other4 = 6;
        break;

    case 31:
        main   = 6;
        other1 = 0;

        other2 = 2;
        other3 = 5;
        other4 = 7;
        break;

    case 32:
        main   = 7;
        other1 = 1;

        other2 = 3;
        other3 = 4;
        other4 = 6;
        break;

    default:
        EMLD_THROW( "Invalid diagonal: " << i_diag );
        break;
    }

    // Diagonals are NOT shared, so the ordering of the corners matters.
    V3T numerP( T(0) );
    V3T numerDpDu( T(0) );
    V3T numerDpDv( T(0) );
    V3T numerDpDw( T(0) );

    T denom( T(0) );

    EMLD_LITERAL_CONSTANT T Sqrt3 = 1.7320508075688772; //std::sqrt( T(3) );
    EMLD_LITERAL_CONSTANT T Sqrt2 = 1.4142135623730951; //std::sqrt( T(2) );

    const V3i cMain = m_corners[main];
    const V3i cOther1 = m_corners[other1];

    const T qMain = m_levelSet[main];
    const T qOther1 = m_levelSet[other1];
    const T qOther2 = m_levelSet[other2];
    const T qOther3 = m_levelSet[other3];
    const T qOther4 = m_levelSet[other4];

    T interp;

    // We compute a weighted average of the usable zero crossings
    // along each of the main->other diagonals to produce a cell-interior
    // point.

    // Long diagonal
    if ( CrossesSurface( qMain, qOther1 ) )
    {
        interp = FindZeroInterpolant( qMain, qOther1 );
        numerP += Sqrt3 * mix( m_p[main], m_p[other1], interp );
        numerDpDu += Sqrt3 * mix( m_dPdu[main], m_dPdu[other1], interp );
        numerDpDv += Sqrt3 * mix( m_dPdv[main], m_dPdv[other1], interp );
        numerDpDw += Sqrt3 * mix( m_dPdw[main], m_dPdw[other1], interp );
        denom += Sqrt3;
    }

    // Adjacent face diagonal 0
    if ( CrossesSurface( qMain, qOther2 ) )
    {
        interp = FindZeroInterpolant( qMain, qOther2 );
        numerP += Sqrt2 * mix( m_p[main], m_p[other2], interp );
        numerDpDu += Sqrt2 * mix( m_dPdu[main], m_dPdu[other2], interp );
        numerDpDv += Sqrt2 * mix( m_dPdv[main], m_dPdv[other2], interp );
        numerDpDw += Sqrt2 * mix( m_dPdw[main], m_dPdw[other2], interp );
        denom += Sqrt2;
    }

    // Adjacent face diagonal 1
    if ( CrossesSurface( qMain, qOther3 ) )
    {
        interp = FindZeroInterpolant( qMain, qOther3 );
        numerP += Sqrt2 * mix( m_p[main], m_p[other3], interp );
        numerDpDu += Sqrt2 * mix( m_dPdu[main], m_dPdu[other3], interp );
        numerDpDv += Sqrt2 * mix( m_dPdv[main], m_dPdv[other3], interp );
        numerDpDw += Sqrt2 * mix( m_dPdw[main], m_dPdw[other3], interp );
        denom += Sqrt2;
    }

    // Adjacent face diagonal 2
    if ( CrossesSurface( qMain, qOther4 ) )
    {
        interp = FindZeroInterpolant( qMain, qOther4 );
        numerP += Sqrt2 * mix( m_p[main], m_p[other4], interp );
        numerDpDu += Sqrt2 * mix( m_dPdu[main], m_dPdu[other4], interp );
        numerDpDv += Sqrt2 * mix( m_dPdv[main], m_dPdv[other4], interp );
        numerDpDw += Sqrt2 * mix( m_dPdw[main], m_dPdw[other4], interp );
        denom += Sqrt2;
    }

    // If we didn't find a single good interior point, we'll just
    // punt and put the diagonal .25 along the distance between main
    // and the primary other point.
    if ( denom < Sqrt2 )
    {
        o_vertex.set( cMain, cOther1,
                      mix( m_p[main], m_p[other1], T(0.25) ),
                      mix( m_dPdu[main], m_dPdu[other1], T(0.25) ),
                      mix( m_dPdv[main], m_dPdv[other1], T(0.25) ),
                      mix( m_dPdw[main], m_dPdw[other1], T(0.25) ) );
    }
    else
    {
        // Otherwise, we use the weighted average and we're happy.
        o_vertex.set( cMain, cOther1,
                      numerP / denom,
                      numerDpDu / denom,
                      numerDpDv / denom,
                      numerDpDw / denom );
    }
}

//-*****************************************************************************
template <typename MBUILD>
void
Cell<MBUILD>::getVertex( int i_edgeNumber, PreVertex<T>& o_vertex ) const
{
    if ( i_edgeNumber < 1 || i_edgeNumber > 32 )
    {
        EMLD_THROW( "ElutCell: Invalid edge number: " << i_edgeNumber );
    }
    else if ( i_edgeNumber < 25 )
    {
        getEdge( i_edgeNumber, o_vertex );
    }
    else
    {
        // 24 < v < 33
        getDiagonal( i_edgeNumber, o_vertex );
    }
}

} // End namespace ElutMesh
} // End namespace EmldCore

#endif

