#pragma once

#include <emerald/wisp/foundation.h>

namespace emerald::wisp {

struct State {
    State() noexcept = default;
    State(int2 const size)
      : PrevU(size)
      , U(size)
      , PrevV(size)
      , V(size)
      , PrevDensityRGB(size)
      , DensityRGB(size)
      , InputU(size)
      , InputV(size)
      , InputDensityRGB(size)
      , Temp0(size)
      , Temp1(size)
      , Temp2(size) {
    }
    ~State() noexcept = default;

    explicit State(State const&) = default;
    State(State&&) noexcept = default;

    State& operator=(State const&) = default;
    State& operator=(State&&) noexcept = default;

    Float_slab PrevU;
    Float_slab U;
    Float_slab PrevV;
    Float_slab V;
    Float3_slab PrevDensityRGB;
    Float3_slab DensityRGB;
    Float_slab InputU;
    Float_slab InputV;
    Float3_slab InputDensityRGB;
    Float_slab Temp0;
    Float_slab Temp1;
    Float_slab Temp2;
};

}  // namespace emerald::wisp
