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
      , PrevDensityR(size)
      , PrevDensityG(size)
      , PrevDensityB(size)
      , DensityR(size)
      , DensityG(size)
      , DensityB(size)
      , InputU(size)
      , InputV(size)
      , InputDensityR(size)
      , InputDensityG(size)
      , InputDensityB(size)
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
    Float_slab PrevDensityR;
    Float_slab PrevDensityG;
    Float_slab PrevDensityB;
    Float_slab DensityR;
    Float_slab DensityG;
    Float_slab DensityB;
    Float_slab InputU;
    Float_slab InputV;
    Float_slab InputDensityR;
    Float_slab InputDensityG;
    Float_slab InputDensityB;
    Float_slab Temp0;
    Float_slab Temp1;
    Float_slab Temp2;
};

}  // namespace emerald::wisp
