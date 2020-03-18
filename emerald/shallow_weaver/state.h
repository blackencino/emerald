#pragma once

#include <emerald/shallow_weaver/foundation.h>
#include <emerald/shallow_weaver/slab.h>

namespace emerald::shallow_weaver {

struct State {
    State() noexcept = default;
    State(int2 const size)
      : height(size)
      , terrain_height(size)
      , velocity(size)
      , height_prev(size)
      , velocity_prev(size)
      , velocity_star(size)
      , acceleration_star(size)
      , height_star(size)
      , jacobi_tmp(size)
      , jacobi_tmp_2(size)
      , jacobi_tmp_3(size)
      , jacobi_tmp_4(size)
      , jacobi_tmp_5(size)
      , jacobi_tmp_6(size)
      , jacobi_tmp_7(size)
      , jacobi_tmp_8(size) {
    }
    ~State() noexcept = default;

    explicit State(State const&) = default;
    State(State&&) noexcept = default;

    State& operator=(State const&) = default;
    State& operator=(State&&) noexcept = default;

    Float_slab height;
    Float_slab terrain_height;
    Float_slab velocity;
    Float_slab height_prev;
    Float_slab velocity_prev;
    Float_slab velocity_star;
    Float_slab acceleration_star;
    Float_slab height_star;
    Float_slab jacobi_tmp;
    Float_slab jacobi_tmp_2;
    Float_slab jacobi_tmp_3;
    Float_slab jacobi_tmp_4;
    Float_slab jacobi_tmp_5;
    Float_slab jacobi_tmp_6;
    Float_slab jacobi_tmp_7;
    Float_slab jacobi_tmp_8;
};

}  // namespace emerald::shallow_weaver
