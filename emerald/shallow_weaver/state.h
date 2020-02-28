#pragma once

#include <emerald/shallow_weaver/slab.h>

namespace emerald::shallow_weaver {

struct State {
    State() noexcept = default;
    State(int2 const size)
      : height(size)
      , velocity(size)
      , height_prev(size)
      , velocity_prev(size)
      , velocity_star(size)
      , acceleration_star(size)
      , height_star(size)
      , jacobi_tmp(size)
      , jacobi_tmp_2(size) {
    }
    ~State() noexcept = default;

    explicit State(State const&) = default;
    State(State&&) noexcept = default;

    State& operator=(State const&) = default;
    State& operator=(State&&) noexcept = default;

    Float_slab height;
    Float_slab velocity;
    Float_slab height_prev;
    Float_slab velocity_prev;
    Float_slab velocity_star;
    Float_slab acceleration_star;
    Float_slab height_star;
    Float_slab jacobi_tmp;
    Float_slab jacobi_tmp_2;
};

}  // namespace emerald::shallow_weaver
