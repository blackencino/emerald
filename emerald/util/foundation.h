#pragma once

#include <Alembic/AbcGeom/All.h>
#include <Alembic/Util/All.h>

#include <OpenEXR/ImathMath.h>
#include <OpenEXR/ImathVec.h>
#include <OpenEXR/half.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace emerald {
namespace util {

// Bring in Alembic types!
namespace AbcG = Alembic::AbcGeom;

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

using Imath::V4d;
using Imath::V4f;
using Imath::V4i;
using Imath::V4s;

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

using AbcG::C4c;
using AbcG::C4f;
using AbcG::C4h;

using AbcG::N3d;
using AbcG::N3f;

//-*****************************************************************************
template <typename T>
struct zero_bits {
    T bits;
    zero_bits() {
        auto* const data = reinterpret_cast<char*>(&bits);
        std::fill(data, data + sizeof(T), (char)0);
    }

    operator T const&() const {
        return bits;
    }

    operator T&() {
        return bits;
    }
};

//-*****************************************************************************
template <typename T>
inline void set_zero(T& o_val) {
    static zero_bits<T> const zero;
    o_val = zero;
}

//-*****************************************************************************
#define EMLD_GOOD_FLOAT(F) (std::isfinite(F))

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
struct AllZeroConstructor {
    AllZeroConstructor() {
        auto* const dthis = static_cast<DERIVED*>(this);
        // memset( ( void* )dthis, 0, sizeof( DERIVED ) );
        auto* const cthis = reinterpret_cast<char*>(dthis);
        std::fill(cthis, cthis + sizeof(DERIVED), 0);
    }
};

}  // End namespace util
}  // End namespace emerald

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
#else
namespace Imath {
#endif

//-*****************************************************************************
template <typename VEC>
std::ostream& operator<<(std::ostream& ostr, Box<VEC> const& i) {
    ostr << "( " << i.min << " to " << i.max << " )";
    return ostr;
}

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
#else
}  // End namespace Imath
#endif
