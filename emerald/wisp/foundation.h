#pragma once

#include <emerald/shallow_weaver/foundation.h>
#include <emerald/shallow_weaver/slab.h>
#include <emerald/util/foundation.h>

#include <array>

namespace emerald::wisp {

using namespace emerald::util;

using emerald::shallow_weaver::float2;
using emerald::shallow_weaver::float4;
using emerald::shallow_weaver::int2;
using emerald::shallow_weaver::int4;
using float3 = std::array<float, 3>;

using emerald::shallow_weaver::Float_slab;
using Float3_slab = emerald::shallow_weaver::Slab<float3>;

}  // namespace emerald::wisp
