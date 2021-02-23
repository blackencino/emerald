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

#ifndef _EmldCore_Util_VectorUtil_h_
#define _EmldCore_Util_VectorUtil_h_

#include "Foundation.h"
#include "Murmur3.h"

namespace EmldCore {
namespace Util {

//-*****************************************************************************
// C++ 11 guarantees that vectors have a "data" member function.
// However, older C++ does not. Create a vector_data function to
// access the data.
#if EMLD_USE_CXX11

template <typename VECTOR>
inline typename VECTOR::pointer vector_data( VECTOR& i_vec )
{
    return i_vec.data();
}

template <typename VECTOR>
inline typename VECTOR::const_pointer vector_cdata( const VECTOR& i_vec )
{
    return i_vec.data();
}

#else

template <typename VECTOR>
inline typename VECTOR::pointer vector_data( VECTOR& i_vec )
{
    if ( i_vec.size() == 0 )
    {
        return reinterpret_cast<typename VECTOR::pointer>( 0 );
    }
    else
    {
        return reinterpret_cast<typename VECTOR::pointer>( &( i_vec.front() ) );
    }
}

template <typename VECTOR>
inline typename VECTOR::const_pointer vector_cdata( const VECTOR& i_vec )
{
    if ( i_vec.size() == 0 )
    {
        return reinterpret_cast<typename VECTOR::const_pointer>( 0 );
    }
    else
    {
        return reinterpret_cast<typename VECTOR::const_pointer>(
                   &( i_vec.front() ) );
    }
}

#endif

//-*****************************************************************************
// FOR COMPUTING THE HASH OF AN ARRAY OF DATA
//-*****************************************************************************

//-*****************************************************************************
struct VectorHashKey
{
    VectorHashKey() { key[0] = 0; key[1] = 0; }
    uint64_t key[2];
};

//-*****************************************************************************
inline bool operator==( const VectorHashKey& a, const VectorHashKey& b )
{
    return ( a.key[0] == b.key[0] ) && ( a.key[1] == b.key[1] );
}

//-*****************************************************************************
inline bool operator!=( const VectorHashKey& a, const VectorHashKey& b )
{
    return ( a.key[0] != b.key[0] ) || ( a.key[1] != b.key[1] );
}

//-*****************************************************************************
inline bool operator<( const VectorHashKey& a, const VectorHashKey& b )
{
    if ( a.key[0] < b.key[0] ) { return true; }
    else if ( a.key[0] > b.key[0] ) { return false; }
    else { return ( a.key[1] < b.key[1] ); }
}

//-*****************************************************************************
inline bool operator<=( const VectorHashKey& a, const VectorHashKey& b )
{
    if ( a.key[0] < b.key[0] ) { return true; }
    else if ( a.key[0] > b.key[0] ) { return false; }
    else { return ( a.key[1] <= b.key[1] ); }
}

//-*****************************************************************************
inline bool operator>( const VectorHashKey& a, const VectorHashKey& b )
{
    if ( a.key[0] > b.key[0] ) { return true; }
    else if ( a.key[0] < b.key[0] ) { return false; }
    else { return ( a.key[1] > b.key[1] ); }
}

//-*****************************************************************************
inline bool operator>=( const VectorHashKey& a, const VectorHashKey& b )
{
    if ( a.key[0] > b.key[0] ) { return true; }
    else if ( a.key[0] < b.key[0] ) { return false; }
    else { return ( a.key[1] >= b.key[1] ); }
}

//-*****************************************************************************
inline std::ostream& operator<<( std::ostream& ostr, const VectorHashKey& key )
{
    ostr << ( boost::format( "%016x--%016x" ) % key.key[0] % key.key[1] ) ;
    return ostr;
}

//-*****************************************************************************
template <typename VECTOR>
inline void ComputeVectorHashKey( const VECTOR& i_array,
                                  VectorHashKey& io_hashKey )
{
    typedef typename VECTOR::value_type value_type;
    typedef storage_traits<value_type> traits_type;
    typedef typename traits_type::pod_type pod_type;

    std::size_t N = i_array.size();
    if ( N > 0 )
    {
        const void* cdata =
            reinterpret_cast<const void*>( vector_cdata( i_array ) );

        MurmurHash3_x64_128( cdata,
                             sizeof( value_type ) * N,
                             sizeof( pod_type ),
                             io_hashKey.key );
    }
}

//-*****************************************************************************
template <typename VECTOR>
inline VectorHashKey ComputeVectorHashKey( const VECTOR& i_array )
{
    VectorHashKey key;
    ComputeVectorHashKey<VECTOR>( i_array, key );
    return key;
}

//-*****************************************************************************
template <typename T>
inline void ComputeDataArrayHashKey
( 
    const T* i_data, 
    std::size_t i_num,
    VectorHashKey& io_hashKey,
    uint32_t i_seed = sizeof( typename storage_traits<T>::pod_type )
)
{
    if ( i_num > 0 )
    {
        const void *cdata = 
            reinterpret_cast<const void*>( i_data );

        MurmurHash3_x64_128( cdata, 
                             sizeof( T ) * i_num,
                             i_seed,
                             io_hashKey.key );
    }
}

//-*****************************************************************************
template <typename T>
inline VectorHashKey 
ComputeDataArrayHashKey
( 
    const T* i_data, 
    std::size_t i_num,
    uint32_t i_seed = sizeof( typename storage_traits<T>::pod_type )
)
{
    VectorHashKey key;
    ComputeDataArrayHashKey<T>( i_data, i_num, key, i_seed );
    return key;
}

} // End namespace Util
} // End namespace EmldCore



#endif
