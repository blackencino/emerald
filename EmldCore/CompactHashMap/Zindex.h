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

#ifndef _EmldCore_CompactHashMap_Zindex_h_
#define _EmldCore_CompactHashMap_Zindex_h_

#include "Foundation.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
// Traits for Zindex class.
template <class UINT>
struct ZindexTraits;

//-*****************************************************************************
template <>
struct ZindexTraits<uint32_t>
{
    typedef uint32_t uint_type;
    typedef int32_t int_type;

    // We can fit 3x10 bits into a 32 bit index.
    // The first 10 bits are 0x000003ff
    CHM_STATIC_CONSTEXPR uint32_t axisSize = 0x1 << 10;
    CHM_STATIC_CONSTEXPR uint32_t axisMask = axisSize - 1U;
};

//-*****************************************************************************
template <>
struct ZindexTraits<uint64_t>
{
    typedef uint64_t uint_type;
    typedef int64_t int_type;

    // We can fit 3x21 bits into a 64 bit index.
    // The first 21 bits are 0x001fffff
    CHM_STATIC_CONSTEXPR uint64_t axisSize = 0x1 << 21;
    CHM_STATIC_CONSTEXPR uint64_t axisMask = axisSize - 1U;
};

//-*****************************************************************************
//-*****************************************************************************
// ZINDEX LUT CLASS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename UINT>
class ZindexLUT
{
public:
    typedef UINT uint_type;
    typedef ZindexTraits<uint_type> traits_type;
    typedef typename traits_type::int_type int_type;

    typedef typename Imath::Vec3<uint_type> v3u_type;
    typedef typename Imath::Vec3<int_type> v3i_type;

    typedef typename Imath::Vec2<uint_type> v2u_type;
    typedef typename Imath::Vec2<int_type> v2i_type;

    ZindexLUT();

    //-*************************************************************************
    // Vec2 operators
    //-*************************************************************************

    uint_type operator[]( const v2u_type& i_cellPt ) const
    {
        CHM_STATIC_CONSTEXPR uint_type CELL_OFFSET_X = 1062599;
        CHM_STATIC_CONSTEXPR uint_type CELL_OFFSET_Y = 3626149;
        return
            m_xAxisIndices[( i_cellPt.x - CELL_OFFSET_X ) &
                           traits_type::axisMask ] |
            ( m_xAxisIndices[( i_cellPt.y - CELL_OFFSET_Y ) &
                             traits_type::axisMask ] << 1 );
    }

    uint_type operator[]( const v2i_type& i_cellPt ) const
    {
        return operator[]
               ( reinterpret_cast< const v2u_type& >( i_cellPt ) );
    }

    //-*************************************************************************
    // Vec3 operators
    //-*************************************************************************

    uint_type operator[]( const v3u_type& i_cellPt ) const
    {
        CHM_STATIC_CONSTEXPR uint_type CELL_OFFSET_X = 1062599;
        CHM_STATIC_CONSTEXPR uint_type CELL_OFFSET_Y = 3626149;
        CHM_STATIC_CONSTEXPR uint_type CELL_OFFSET_Z = 4393139;
        return
            m_xAxisIndices[( i_cellPt.x - CELL_OFFSET_X ) &
                           traits_type::axisMask ] |
            ( m_xAxisIndices[( i_cellPt.y - CELL_OFFSET_Y ) &
                             traits_type::axisMask ] << 1 ) |
            ( m_xAxisIndices[( i_cellPt.z - CELL_OFFSET_Z ) &
                             traits_type::axisMask ] << 2 );
    }

    uint_type operator[]( const v3i_type& i_cellPt ) const
    {
        return operator[]
               ( reinterpret_cast< const v3u_type& >( i_cellPt ) );
    }


protected:
    std::vector<uint_type> m_xAxisIndices;
};


//-*****************************************************************************
//-*****************************************************************************
template <typename UINT>
static inline UINT XaxisIndex( UINT a );

//-*****************************************************************************
// Specialization for 32-bit
//-*****************************************************************************
template <>
uint32_t XaxisIndex<uint32_t>( uint32_t x )
{
    // Use the low 10 bits of x, with the first bit in the 0 position
    return
        ( ( x & ( 0x1 << 0 ) ) << 0 ) |
        ( ( x & ( 0x1 << 1 ) ) << 2 ) |
        ( ( x & ( 0x1 << 2 ) ) << 4 ) |
        ( ( x & ( 0x1 << 3 ) ) << 6 ) |
        ( ( x & ( 0x1 << 4 ) ) << 8 ) |
        ( ( x & ( 0x1 << 5 ) ) << 10 ) |
        ( ( x & ( 0x1 << 6 ) ) << 12 ) |
        ( ( x & ( 0x1 << 7 ) ) << 14 ) |
        ( ( x & ( 0x1 << 8 ) ) << 16 ) |
        ( ( x & ( 0x1 << 9 ) ) << 18 ) ;
}

//-*****************************************************************************
// Specialization for 64-bit
//-*****************************************************************************
template <>
uint64_t XaxisIndex<uint64_t>( uint64_t x )
{
    // Use the low 21 bits of x, with the first bit in the 0 position
    return
        ( ( x & ( 0x1 << 0 ) ) << 0 ) |
        ( ( x & ( 0x1 << 1 ) ) << 2 ) |
        ( ( x & ( 0x1 << 2 ) ) << 4 ) |
        ( ( x & ( 0x1 << 3 ) ) << 6 ) |
        ( ( x & ( 0x1 << 4 ) ) << 8 ) |
        ( ( x & ( 0x1 << 5 ) ) << 10 ) |
        ( ( x & ( 0x1 << 6 ) ) << 12 ) |
        ( ( x & ( 0x1 << 7 ) ) << 14 ) |
        ( ( x & ( 0x1 << 8 ) ) << 16 ) |
        ( ( x & ( 0x1 << 9 ) ) << 18 ) |

        ( ( x & ( 0x1 << 10 ) ) << 20 ) |
        ( ( x & ( 0x1 << 11 ) ) << 22 ) |
        ( ( x & ( 0x1 << 12 ) ) << 24 ) |
        ( ( x & ( 0x1 << 13 ) ) << 26 ) |
        ( ( x & ( 0x1 << 14 ) ) << 28 ) |
        ( ( x & ( 0x1 << 15 ) ) << 30 ) |
        ( ( x & ( 0x1 << 16 ) ) << 32 ) |
        ( ( x & ( 0x1 << 17 ) ) << 34 ) |
        ( ( x & ( 0x1 << 18 ) ) << 36 ) |
        ( ( x & ( 0x1 << 19 ) ) << 38 ) |

        ( ( x & ( 0x1 << 20 ) ) << 40 ) ;
}

//-*****************************************************************************
//-*****************************************************************************
template <typename UINT>
struct ApplyXaxisIndex
        : public ParallelUtil::ZeroForEachFunctorI
        <ApplyXaxisIndex<UINT>,
         typename ZindexTraits<UINT>::int_type>
{
    typedef UINT uint_type;
    typedef typename ZindexTraits<UINT>::int_type index_type;

    uint_type* X;

    void operator()( index_type i ) const
    {
        X[i] = XaxisIndex<uint_type>( static_cast<uint_type>( i ) );
    }
};

//-*****************************************************************************
//-*****************************************************************************
// Build some big-ass tables for X.
//-*****************************************************************************
//-*****************************************************************************
template <typename UINT>
ZindexLUT<UINT>::ZindexLUT()
    : m_xAxisIndices( ZindexTraits<UINT>::axisSize )
{
    // -----------
    {
        ApplyXaxisIndex<UINT> F;
        F.X = vector_data( m_xAxisIndices );
        F.execute( m_xAxisIndices.size() );
    }
    //std::cout << "Created Zindex x indices" << std::endl;
}

//-*****************************************************************************
// Global Zindex instance.
extern ZindexLUT<uint32_t> g_zIndexLUT32;
extern ZindexLUT<uint64_t> g_zIndexLUT64;

//-*****************************************************************************
// VEC2 Zindex Function
//-*****************************************************************************

//-*****************************************************************************
inline uint32_t Zindex( const Imath::Vec2<uint32_t>& i_v )
{ return g_zIndexLUT32[ i_v ]; }

//-*****************************************************************************
inline uint32_t Zindex( const Imath::Vec2<int32_t>& i_v )
{ return g_zIndexLUT32[ i_v ]; }

//-*****************************************************************************
inline uint64_t Zindex( const Imath::Vec2<uint64_t>& i_v )
{ return g_zIndexLUT64[ i_v ]; }

//-*****************************************************************************
inline uint64_t Zindex( const Imath::Vec2<int64_t>& i_v )
{ return g_zIndexLUT64[ i_v ]; }

//-*****************************************************************************
// VEC3 Zindex Function
//-*****************************************************************************

//-*****************************************************************************
inline uint32_t Zindex( const Imath::Vec3<uint32_t>& i_v )
{ return g_zIndexLUT32[ i_v ]; }

//-*****************************************************************************
inline uint32_t Zindex( const Imath::Vec3<int32_t>& i_v )
{ return g_zIndexLUT32[ i_v ]; }

//-*****************************************************************************
inline uint64_t Zindex( const Imath::Vec3<uint64_t>& i_v )
{ return g_zIndexLUT64[ i_v ]; }

//-*****************************************************************************
inline uint64_t Zindex( const Imath::Vec3<int64_t>& i_v )
{ return g_zIndexLUT64[ i_v ]; }


} // End namespace CompactHashMap
} // End namespace EmldCore

#endif
