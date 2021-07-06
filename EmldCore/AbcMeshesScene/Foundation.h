#ifndef _EmldCore_AbcMeshesScene_Foundation_h_
#define _EmldCore_AbcMeshesScene_Foundation_h_

#define ABCM_USE_CXX11 1

#pragma warning(push)
#pragma warning( disable : 4244 4100 4456 )

#include <EmldCore/ParallelUtil/All.h>
#include <EmldCore/TriMesh/All.h>
#include <EmldCore/Util/All.h>

#include <Alembic/Abc/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/Util/All.h>

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathBoxAlgo.h>
#include <OpenEXR/ImathColor.h>
#include <OpenEXR/ImathFun.h>
#include <OpenEXR/ImathMath.h>
#include <OpenEXR/ImathMatrix.h>
#include <OpenEXR/ImathQuat.h>
#include <OpenEXR/ImathRandom.h>
#include <OpenEXR/ImathVec.h>

#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>

#include <chrono>
#include <memory>
#include <random>
#include <ratio>
#include <regex>
#include <type_traits>
#include <unordered_map>

#define ABCM_SHARED_PTR std::shared_ptr
#define ABCM_UNIQUE_PTR std::unique_ptr
#define ABCM_STATIC_CONSTEXPR static constexpr

#include <stdlib.h>
#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

//-*****************************************************************************

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <limits>

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
#define ABCM_THROW(TEXT) EMLD_FAIL(TEXT)
#define ABCM_ASSERT(COND, TEXT) EMLD_ASSERT(COND, TEXT)
#define ABCM_WARN(TEXT)                                \
    do {                                               \
        std::stringstream sstr;                        \
        sstr << TEXT;                                  \
        sstr << "\nFile: " << __FILE__ << std::endl    \
             << "Line: " << __LINE__ << std::endl;     \
        std::cerr << "WARNING: " << sstr << std::endl; \
    } while (0)

//-*****************************************************************************
// Bring in Alembic types!

using AbcG::bool_t;
using AbcG::byte_t;
using AbcG::chrono_t;
using AbcG::float16_t;
using AbcG::float32_t;
using AbcG::float64_t;
using AbcG::index_t;
using AbcG::int16_t;
using AbcG::int32_t;
using AbcG::int64_t;
using AbcG::int8_t;
using AbcG::uint16_t;
using AbcG::uint32_t;
using AbcG::uint64_t;
using AbcG::uint8_t;

using AbcG::V2d;
using AbcG::V2f;
using AbcG::V2i;
using AbcG::V2s;

using AbcG::V3d;
using AbcG::V3f;
using AbcG::V3i;
using AbcG::V3s;

using AbcG::Box2d;
using AbcG::Box2f;
using AbcG::Box2i;
using AbcG::Box2s;

using AbcG::Box3d;
using AbcG::Box3f;
using AbcG::Box3i;
using AbcG::Box3s;

using AbcG::M33d;
using AbcG::M33f;
using AbcG::M44d;
using AbcG::M44f;

using AbcG::Quatd;
using AbcG::Quatf;

using AbcG::C3c;
using AbcG::C3f;
using AbcG::C3h;
typedef Imath::Color3<double> C3d;

using AbcG::C4c;
using AbcG::C4f;
using AbcG::C4h;
typedef Imath::Color4<double> C4d;

using AbcG::N3d;
using AbcG::N3f;

//-*****************************************************************************
inline Box3f toBox3f(const Box3d& i_b) {
    if (i_b.isEmpty()) {
        Box3f r;
        r.makeEmpty();
        return r;
    } else {
        return Box3f(V3f(i_b.min), V3f(i_b.max));
    }
}

//-*****************************************************************************
inline Box3d toBox3d(const Box3f& i_b) {
    if (i_b.isEmpty()) {
        Box3d r;
        r.makeEmpty();
        return r;
    } else {
        return Box3d(V3d(i_b.min), V3d(i_b.max));
    }
}

}  // End namespace AbcMeshesScene
}  // End namespace EmldCore


#pragma warning(pop)

#endif
