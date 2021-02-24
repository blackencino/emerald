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

#ifndef _EmldCore_ElutMesh_PreVertex_h_
#define _EmldCore_ElutMesh_PreVertex_h_

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

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
// We generate vertices from a pair of corner indices, expressed as V3i's,
// which are assumed to be ordered by the calling functions such that
// shared edges will have the same ordering.  In other words, the
// given two cell coordinates cA & cB, the PreVertex cA,cB is a different
// vertex than the PreVertex cB,cA.  When these are being constructed,
// If they're aligned to a shared edge, the calling application should
// take care to sort the coordinates before creating the edge, ensuring
// that the edge will not be shared.  Conversely, when the pre vertex is
// part of an interior diagonal to a cell, we sometimes need to create both
// directions of the diagonal, which will sometimes evaluate to different
// final spatial locations.
// This is also the reason why we explicitly pass in a world-space coordinate
// for the edge, though it is not used when making equality comparisons. 
template <typename T>
class PreVertex
{
public:
    typedef Imath::Vec3<T> V3T;

    PreVertex()
        : m_position( T( 0 ) )
    {
        m_corners[0] = V3i( 0 );
        m_corners[1] = V3i( 0 );
    }

    PreVertex( const V3i& i_c0, 
               const V3i& i_c1, 
               const V3T& i_p,
               const V3T& i_dPdu,
               const V3T& i_dPdv,
               const V3T& i_dPdw )
        : m_position( i_p )
        , m_dPdu( i_dPdu )
        , m_dPdv( i_dPdv )
        , m_dPdw( i_dPdw )
    {
        m_corners[0] = i_c0;
        m_corners[1] = i_c1;
        EMLD_DEBUG_ASSERT( m_corners[0] != m_corners[1], "Degenerate edge" );
    }

    // Like calling the constructor
    void set( const V3i& i_c0, const V3i& i_c1, 
              const V3T& i_p,
              const V3T& i_dPdu, const V3T& i_dPdv, const V3T& i_dPdw )
    {
        m_corners[0] = i_c0;
        m_corners[1] = i_c1;
        EMLD_DEBUG_ASSERT( m_corners[0] != m_corners[1], "Degenerate edge" );
        m_position = i_p;
        m_dPdu = i_dPdu;
        m_dPdv = i_dPdv;
        m_dPdw = i_dPdw;
    }

    void get( V3T& o_p, 
              V3T& o_dPdu, V3T& o_dPdv, V3T& o_dPdw ) const
    {
        o_p = m_position;
        o_dPdu = m_dPdu;
        o_dPdv = m_dPdv;
        o_dPdw = m_dPdw;
    }

    void get( V3i& o_c0, V3i& o_c1,
              V3T& o_p, 
              V3T& o_dPdu, V3T& o_dPdv, V3T& o_dPdw ) const
    {
        o_c0 = m_corners[0];
        o_c1 = m_corners[1];
        o_p = m_position;
        o_dPdu = m_dPdu;
        o_dPdv = m_dPdv;
        o_dPdw = m_dPdw;
    }

    const V3i& corner0() const { return m_corners[0]; }
    const V3i& corner1() const { return m_corners[1]; }
    const V3T& position() const { return m_position; }
    const V3T& dPdu() const { return m_dPdu; }
    const V3T& dPdv() const { return m_dPdv; }
    const V3T& dPdw() const { return m_dPdw; }

    // A hash function.
    std::size_t hash() const
    {
        return CompactHashMap::SpatialHash( m_corners[0] ) +
            CompactHashMap::SpatialHash( m_corners[1] );
    }

    // As a functor
    struct HashF
    {
        std::size_t operator()( const PreVertex<T>& i_pv ) const 
        {
            return i_pv.hash();
        }
    };

    // Equality functor
    struct EqualF
    {
        bool operator()( const PreVertex<T>& i_a, 
                         const PreVertex<T>& i_b ) const
        {
            return ( i_a.corner0() == i_b.corner0() ) &&
                    ( i_a.corner1() == i_b.corner1() );
        }
    };

protected:
    V3i m_corners[2];
    V3T m_position;
    V3T m_dPdu;
    V3T m_dPdv;
    V3T m_dPdw;
};

//-*****************************************************************************
// The equality logic is a little weird for pre vertices.
// We ignore the explicit position, just looking at corner indices.
template <typename T>
inline bool operator==( const PreVertex<T>& i_a, const PreVertex<T>& i_b )
{
    return ( i_a.corner0() == i_b.corner0() ) &&
            ( i_a.corner1() == i_b.corner1() );
}

//-*****************************************************************************
template <typename T>
inline bool operator!=( const PreVertex<T>& i_a, const PreVertex<T>& i_b )
{
    return ( i_a.corner0() != i_b.corner0() ) ||
            ( i_a.corner1() != i_b.corner1() );
}

//-*****************************************************************************
template <typename T>
inline bool operator<( const PreVertex<T>& i_a, const PreVertex<T>& i_b )
{
    if ( i_a.corner0() < i_b.corner0() ) { return true; }
    else if ( i_a.corner0() > i_b.corner0() ) { return false; }
    else { return i_a.corner1() < i_b.corner1(); }
}

//-*****************************************************************************
template <typename T>
inline bool operator<=( const PreVertex<T>& i_a, const PreVertex<T>& i_b )
{
    // Not a typo here, only use <= in the last comparator
    if ( i_a.corner0() < i_b.corner0() ) { return true; }
    else if ( i_a.corner0() > i_b.corner0() ) { return false; }
    else { return i_a.corner1() <= i_b.corner1(); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>( const PreVertex<T>& i_a, const PreVertex<T>& i_b )
{
    if ( i_a.corner0() > i_b.corner0() ) { return true; }
    else if ( i_a.corner0() < i_b.corner0() ) { return false; }
    else { return i_a.corner1() > i_b.corner1(); }
}

//-*****************************************************************************
template <typename T>
inline bool operator>=( const PreVertex<T>& i_a, const PreVertex<T>& i_b )
{
    // Not a typo here, only use >= in the last comparator
    if ( i_a.corner0() > i_b.corner0() ) { return true; }
    else if ( i_a.corner0() < i_b.corner0() ) { return false; }
    else { return i_a.corner1() >= i_b.corner1(); }
}
        
} // End namespace ElutMesh 
} // End namespace EmldCore


#endif
