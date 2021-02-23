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

#ifndef _EmldCore_Util_Foundation_h_
#define _EmldCore_Util_Foundation_h_

//-*****************************************************************************
// CHECK TO SEE IF WE SHOULD USE C++ 11 STUFF
#ifndef EMLD_USE_CXX11

#if __cplusplus > 199711L
#define EMLD_USE_CXX11 1
#else
#undef EMLD_USE_CXX11
#endif

#endif

// CJH NOTE: This is from a specific branch of field 3d.
#ifndef EMLD_F3D_USE_MIPFIELD
#define EMLD_F3D_USE_MIPFIELD 1
#endif

#include <Field3D/ClassFactory.h>
#include <Field3D/Curve.h>
#include <Field3D/DenseField.h>
#include <Field3D/DenseFieldIO.h>
#include <Field3D/Documentation.h>
#include <Field3D/EmptyField.h>
#include <Field3D/Exception.h>
#include <Field3D/Field.h>
#include <Field3D/Field3DFile.h>
#include <Field3D/FieldIO.h>
#include <Field3D/FieldInterp.h>
#include <Field3D/FieldMapping.h>
#include <Field3D/FieldMappingIO.h>
#include <Field3D/FieldMetadata.h>
#include <Field3D/Hdf5Util.h>
#include <Field3D/InitIO.h>
#include <Field3D/Log.h>
#include <Field3D/MACField.h>
#include <Field3D/MACFieldIO.h>
#include <Field3D/MACFieldUtil.h>
#include <Field3D/PluginLoader.h>
#include <Field3D/ProceduralField.h>
#include <Field3D/RefCount.h>
#include <Field3D/SparseDataReader.h>
#include <Field3D/SparseField.h>
#include <Field3D/SparseFieldIO.h>
#include <Field3D/SparseFile.h>
#include <Field3D/Traits.h>
#include <Field3D/Types.h>
#include <Field3D/ns.h>

#if EMLD_F3D_USE_MIPFIELD
#include <Field3D/MIPDenseField.h>
#include <Field3D/MIPDenseFieldIO.h>
#include <Field3D/MIPField.h>
#include <Field3D/MIPSparseField.h>
#include <Field3D/MIPSparseFieldIO.h>
#include <Field3D/MIPUtil.h>
#endif

#include <Alembic/AbcGeom/All.h>
#include <Alembic/Util/All.h>

#if EMLD_USE_CXX11

#include <memory>
#include <functional>
#include <type_traits>

#define EMLD_SHARED_PTR std::shared_ptr
#define EMLD_WEAK_PTR std::weak_ptr
#define EMLD_UNIQUE_PTR std::unique_ptr
#define EMLD_LITERAL_CONSTANT constexpr
#define EMLD_STATLIT_CONSTANT static constexpr
#define EMLD_HASH std::hash

#else

#include <boost/smart_ptr.hpp>
#include <boost/functional/hash.hpp>

#define EMLD_SHARED_PTR boost::shared_ptr
#define EMLD_WEAK_PTR boost::weak_ptr
#define EMLD_UNIQUE_PTR boost::scoped_ptr
#define EMLD_LITERAL_CONSTANT static const
#define EMLD_STATLIT_CONSTANT static const
#define EMLD_HASH boost::hash

#endif

#include <boost/static_assert.hpp>
#include <boost/utility.hpp>
#include <boost/array.hpp>
#include <boost/operators.hpp>
#include <boost/foreach.hpp>
#include <boost/pool/singleton_pool.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/random.hpp>

#include <ImathMath.h>
#include <ImathVec.h>

#include <half.h>

#include <iostream>
#include <sstream>
#include <exception>
#include <limits>
#include <cmath>

#include <map>
#include <string>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

//-*****************************************************************************
namespace EmldCore {
namespace Util {

//-*****************************************************************************
// Bring in Alembic types!
namespace AbcG = Alembic::AbcGeom;

using AbcG::chrono_t;
using AbcG::uint8_t;
using AbcG::int8_t;
using AbcG::uint16_t;
using AbcG::int16_t;
using AbcG::uint32_t;
using AbcG::int32_t;
using AbcG::uint64_t;
using AbcG::int64_t;
using AbcG::float16_t;
using AbcG::float32_t;
using AbcG::float64_t;
using AbcG::bool_t;
using AbcG::byte_t;
using AbcG::index_t;

using AbcG::V2s;
using AbcG::V2i;
using AbcG::V2f;
using AbcG::V2d;

using AbcG::V3s;
using AbcG::V3i;
using AbcG::V3f;
using AbcG::V3d;

using Imath::V4s;
using Imath::V4i;
using Imath::V4f;
using Imath::V4d;

using AbcG::Box2s;
using AbcG::Box2i;
using AbcG::Box2f;
using AbcG::Box2d;

using AbcG::Box3s;
using AbcG::Box3i;
using AbcG::Box3f;
using AbcG::Box3d;

using AbcG::M33f;
using AbcG::M33d;
using AbcG::M44f;
using AbcG::M44d;

using AbcG::Quatf;
using AbcG::Quatd;

using AbcG::C3h;
using AbcG::C3f;
using AbcG::C3c;

using AbcG::C4h;
using AbcG::C4f;
using AbcG::C4c;

using AbcG::N3f;
using AbcG::N3d;

//-*****************************************************************************
template <typename T>
struct zero_bits
{
    T bits;
    zero_bits()
    {
        char* data = reinterpret_cast<char*>( &bits );
        std::fill( data, data + sizeof( T ), ( char )0 );
    }

    operator T const &() const { return bits; }

    operator T&() { return bits; }
};

//-*****************************************************************************
template <typename T>
inline void set_zero( T& o_val )
{
    static const zero_bits<T> zero;
    o_val = zero;
}


//-*****************************************************************************
#define EMLD_GOOD_FLOAT( F ) (std::isfinite(F))

//-*****************************************************************************
// I'm a believer in the 'let's replace Pi with Tau' movement. Tau = 2 Pi.
// It makes wave stuff a LOT cleaner.
#ifndef M_TAU
#define M_TAU 6.2831853071795862
#endif

//-*****************************************************************************
//-*****************************************************************************
//-*****************************************************************************
// For use with the Curiously Recurring Template Pattern, if your class
// can be initialized entirely to bitwise zero, just derive from this class.
// Can't have virtual members.
template <typename DERIVED>
struct AllZeroConstructor
{
    AllZeroConstructor()
    {
        DERIVED* dthis = static_cast<DERIVED*>( this );
        //memset( ( void* )dthis, 0, sizeof( DERIVED ) );
        char* cthis = reinterpret_cast<char*>( dthis );
        std::fill( cthis, cthis + sizeof( DERIVED ), 0 );
    }
};

} // End namespace Util
} // End namespace EmldCore

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
#else
namespace Imath {
#endif

//-*****************************************************************************
template <typename VEC>
std::ostream& operator<<( std::ostream& ostr, const Box<VEC>& i )
{
    ostr << "( " << i.min << " to " << i.max << " )";
    return ostr;
}

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
#else
} // End namespace Imath
#endif

#endif
