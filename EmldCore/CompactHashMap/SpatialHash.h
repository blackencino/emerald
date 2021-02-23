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

#ifndef _EmldCore_CompactHashMap_SpatialHash_h_
#define _EmldCore_CompactHashMap_SpatialHash_h_

#include "Foundation.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
// These large prime numbers used for hashing come from the paper,
// "Optimized Spatial Hashing for Collision Detection of Deformable Objects",
// by Matthias Tescher, Bruno Heidelberger, Matthias Muller,
// Danat Pomeranets, and Markus Gross
inline std::size_t SpatialHash( std::size_t i_x, std::size_t i_y )
{
    EMLD_LITERAL_CONSTANT std::size_t p1 = 73856093;
    EMLD_LITERAL_CONSTANT std::size_t p2 = 19349663;

    return ( i_x * p1 ) ^
           ( i_y * p2 );
}

//-*****************************************************************************
inline std::size_t SpatialHash( std::size_t i_x, 
                                std::size_t i_y, 
                                std::size_t i_z )
{
    EMLD_LITERAL_CONSTANT std::size_t p1 = 73856093;
    EMLD_LITERAL_CONSTANT std::size_t p2 = 19349663;
    EMLD_LITERAL_CONSTANT std::size_t p3 = 83492791;

    return ( i_x * p1 ) ^
           ( i_y * p2 ) ^
           ( i_z * p3 );
}

//-*****************************************************************************
// 8-bit 
//-*****************************************************************************

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<uint8_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<int8_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint8_t&>( i_cell.x ),
                        reinterpret_cast<const uint8_t&>( i_cell.y ) );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<uint8_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y, i_cell.z );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<int8_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint8_t&>( i_cell.x ),
                        reinterpret_cast<const uint8_t&>( i_cell.y ),
                        reinterpret_cast<const uint8_t&>( i_cell.z ) );
}

//-*****************************************************************************
// 16-bit 
//-*****************************************************************************

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<uint16_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<int16_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint16_t&>( i_cell.x ),
                        reinterpret_cast<const uint16_t&>( i_cell.y ) );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<uint16_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y, i_cell.z );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<int16_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint16_t&>( i_cell.x ),
                        reinterpret_cast<const uint16_t&>( i_cell.y ),
                        reinterpret_cast<const uint16_t&>( i_cell.z ) );
}


//-*****************************************************************************
// 32-bit 
//-*****************************************************************************

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<uint32_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<int32_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint32_t&>( i_cell.x ),
                        reinterpret_cast<const uint32_t&>( i_cell.y ) );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<uint32_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y, i_cell.z );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<int32_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint32_t&>( i_cell.x ),
                        reinterpret_cast<const uint32_t&>( i_cell.y ),
                        reinterpret_cast<const uint32_t&>( i_cell.z ) );
}

//-*****************************************************************************
// 64-bit 
//-*****************************************************************************

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<uint64_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec2<int64_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint64_t&>( i_cell.x ),
                        reinterpret_cast<const uint64_t&>( i_cell.y ) );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<uint64_t>& i_cell )
{
    return SpatialHash( i_cell.x, i_cell.y, i_cell.z );
}

//-*****************************************************************************
inline std::size_t SpatialHash( const Imath::Vec3<int64_t>& i_cell )
{
    return SpatialHash( reinterpret_cast<const uint64_t&>( i_cell.x ),
                        reinterpret_cast<const uint64_t&>( i_cell.y ),
                        reinterpret_cast<const uint64_t&>( i_cell.z ) );
}

//-*****************************************************************************
// FUNCTOR
//-*****************************************************************************
template <typename VEC>
struct SpatialHashFunctor
{
    std::size_t operator()( const VEC& i_vec ) const
    {
        return SpatialHash( i_vec );
    }
};
    
} // End namespace CompactHashMap
} // End namespace EmldCore

#endif

