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

#ifndef _EmldCore_Util_StorageTraits_h_
#define _EmldCore_Util_StorageTraits_h_

#include "Foundation.h"

namespace EmldCore {
namespace Util {

//-*****************************************************************************
// A traits class is a mechanism that we assume will be overridden with
// template specializations.
// When we make arrays of data for simulations, we often want to alias
// the same memory for different interpreted types
// So, for example, we might use the same storage for an array of V3f's
// as we do for an array of V3i's, and so on.
// Additionally, we sometimes want to be able to perform really generic
// low-level functions on arrays of data, such as computing a hash-key,
// as alembic does.
// So - we have this traits class which provides a typedef for the value_type,
// the storage_type, the number of bytes, and the POD_TYPE of the primary
// value_type, where POD_TYPE has the usual meaning.
//-*****************************************************************************

template <typename T>
struct storage_traits
{
    typedef T value_type;
    typedef T storage_type;
    typedef T pod_type;
};

//-*****************************************************************************
// Common storage types, expressed via the largest pod.
#pragma pack( push, 1 )
typedef uint8_t                                 storage_bytes_1;
typedef uint16_t                                storage_bytes_2;
typedef struct { uint8_t __data[3]; }           storage_bytes_3;
typedef uint32_t                                storage_bytes_4;
typedef struct { uint16_t __data[3]; }          storage_bytes_6;
typedef uint64_t                                storage_bytes_8;
typedef struct { uint32_t __data[3]; }          storage_bytes_12;
typedef struct { uint64_t __data[2]; }          storage_bytes_16;
typedef struct { uint64_t __data[3]; }          storage_bytes_24;
typedef struct { uint64_t __data[4]; }          storage_bytes_32;
typedef struct { uint32_t __data[9]; }          storage_bytes_36;
typedef struct { uint64_t __data[5]; }          storage_bytes_40;
typedef struct { uint64_t __data[6]; }          storage_bytes_48;
typedef struct { uint32_t __data[13]; }         storage_bytes_52;
typedef struct { uint64_t __data[7]; }          storage_bytes_56;
typedef struct { uint64_t __data[8]; }          storage_bytes_64;
typedef struct { uint64_t __data[9]; }          storage_bytes_72;
typedef struct { uint64_t __data[10]; }         storage_bytes_80;
typedef struct { uint64_t __data[12]; }         storage_bytes_96;
typedef struct { uint64_t __data[16]; }         storage_bytes_128;
typedef struct { uint64_t __data[32]; }         storage_bytes_256;
#pragma pack( pop )

//-*****************************************************************************
// Specializations
#define EMLD_SPECIALIZE_STORAGE_TRAITS( VAL, STORE, POD )  \
template <>                                                   \
struct storage_traits< VAL >                                  \
{                                                             \
    typedef VAL value_type;                                   \
    typedef STORE storage_type;                               \
    typedef POD pod_type;                                     \
}

//-*****************************************************************************
EMLD_SPECIALIZE_STORAGE_TRAITS( uint8_t, storage_bytes_1, uint8_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( int8_t,  storage_bytes_1, int8_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( uint16_t, storage_bytes_2, uint16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( int16_t, storage_bytes_2, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( uint32_t, storage_bytes_4, uint32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( int32_t, storage_bytes_4, int32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( uint64_t, storage_bytes_8, uint64_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( int64_t, storage_bytes_8, int64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( float16_t, storage_bytes_2, float16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( float32_t, storage_bytes_4, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( float64_t, storage_bytes_8, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( V2s, storage_bytes_4, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( V2i, storage_bytes_8, int32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( V2f, storage_bytes_8, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( V2d, storage_bytes_16, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( Imath::Vec2<float16_t>, 
                                   storage_bytes_4, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Imath::Vec2<uint32_t>, 
                                   storage_bytes_8, uint32_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( V3s, storage_bytes_6, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( V3i, storage_bytes_12, int32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( V3f, storage_bytes_12, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( V3d, storage_bytes_24, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( Imath::Vec3<float16_t>, 
                                   storage_bytes_6, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Imath::Vec3<uint32_t>, 
                                   storage_bytes_12, uint32_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( Box2s, storage_bytes_8, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Box2i, storage_bytes_16, int32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Box2f, storage_bytes_16, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Box2d, storage_bytes_32, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( Box3s, storage_bytes_12, int16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Box3i, storage_bytes_24, int32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Box3f, storage_bytes_24, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Box3d, storage_bytes_48, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( M33f, storage_bytes_36, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( M33d, storage_bytes_72, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( M44f, storage_bytes_64, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( M44d, storage_bytes_128, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( Quatf, storage_bytes_16, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( Quatd, storage_bytes_32, float64_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( C3h, storage_bytes_6, float16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( C3f, storage_bytes_12, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( C3c, storage_bytes_3, uint8_t );

EMLD_SPECIALIZE_STORAGE_TRAITS( C4h, storage_bytes_6, float16_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( C4f, storage_bytes_12, float32_t );
EMLD_SPECIALIZE_STORAGE_TRAITS( C4c, storage_bytes_3, uint8_t );

//EMLD_SPECIALIZE_STORAGE_TRAITS( N3f, storage_bytes_12, float32_t );
//EMLD_SPECIALIZE_STORAGE_TRAITS( N3d, storage_bytes_24, float64_t );
   
} // End namespace Util    
} // End namespace EmldCore


#endif
