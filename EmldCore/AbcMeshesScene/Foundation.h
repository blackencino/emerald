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

#ifndef _EmldCore_AbcMeshesScene_Foundation_h_
#define _EmldCore_AbcMeshesScene_Foundation_h_

#if __cplusplus > 199711L
#define ABCM_USE_CXX11 1
#else
#undef ABCM_USE_CXX11
#endif

#include <EmldCore/TriMesh/All.h>
#include <EmldCore/ParallelUtil/All.h>
#include <EmldCore/Util/All.h>

#include <Alembic/AbcGeom/All.h>
#include <Alembic/Abc/All.h>
#include <Alembic/AbcCoreHDF5/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/Util/All.h>

#include <ImathMath.h>
#include <ImathVec.h>
#include <ImathMatrix.h>
#include <ImathBox.h>
#include <ImathQuat.h>
#include <ImathColor.h>
#include <ImathFun.h>
#include <ImathRandom.h>
#include <ImathBoxAlgo.h>

#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/erase.hpp>

#if ABCM_USE_CXX11

#include <unordered_map>
#include <regex>
#include <chrono>
#include <ratio>
#include <random>
#include <memory>
#include <type_traits>

#define ABCM_SHARED_PTR std::shared_ptr
#define ABCM_UNIQUE_PTR std::unique_ptr
#define ABCM_STATIC_CONSTEXPR static constexpr

#else

#include <boost/smart_ptr.hpp>
#include <boost/regex.hpp>
#include <boost/random.hpp>
#include <boost/chrono.hpp>

#define ABCM_SHARED_PTR boost::shared_ptr
#define ABCM_UNIQUE_PTR boost::scoped_ptr
#define ABCM_STATIC_CONSTEXPR static const

#endif

#include <iostream>
#include <vector>
#include <map>
#include <deque>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <utility>
#include <stdlib.h>

//-*****************************************************************************

#include <sys/types.h>
#include <stdio.h>
#include <limits>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
//-*****************************************************************************
// IMPORT SOME STUFF - as long as it's scoped, it's okay.
using namespace ParallelUtil;
using namespace Util;
namespace Etm = EmldCore::TriMesh;
using Etm::TriMesh;

//-*****************************************************************************
// Adopt exceptions
#define ABCM_THROW( TEXT ) EMLD_THROW( TEXT )
#define ABCM_ASSERT( COND, TEXT ) EMLD_ASSERT( COND, TEXT )
#define ABCM_DEBUG_ASSERT( COND, TEXT ) EMLD_DEBUG_ASSERT( COND, TEXT )
#define ABCM_WARN( TEXT )                              \
do                                                    \
{                                                     \
    std::stringstream sstr;                           \
    sstr << TEXT;                                     \
    sstr << "\nFile: " << __FILE__ << std::endl       \
         << "Line: " << __LINE__ << std::endl;        \
    std::cerr << "WARNING: " << sstr << std::endl;    \
}                                                     \
while( 0 )

//-*****************************************************************************
// Bring in Alembic types!

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
typedef Imath::Color3<double> C3d;

using AbcG::C4h;
using AbcG::C4f;
using AbcG::C4c;
typedef Imath::Color4<double> C4d;

using AbcG::N3f;
using AbcG::N3d;

//-*****************************************************************************
inline Box3f toBox3f( const Box3d& i_b )
{ 
    if ( i_b.isEmpty() ) { Box3f r; r.makeEmpty(); return r; }
    else { return Box3f( V3f( i_b.min ), V3f( i_b.max ) ); }
}

//-*****************************************************************************
inline Box3d toBox3d( const Box3f& i_b )
{ 
    if ( i_b.isEmpty() ) { Box3d r; r.makeEmpty(); return r; }
    else { return Box3d( V3d( i_b.min ), V3d( i_b.max ) ); }
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif