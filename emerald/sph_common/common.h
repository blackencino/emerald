#pragma once

#include <emerald/sph_common/types.h>

#include <tbb/parallel_for.h>

#include <algorithm>
#include <utility>

namespace emerald::sph_common {

constexpr bool DO_PARALLEL = true;

//------------------------------------------------------------------------------
// Generics
template <typename Function>
void for_each_iota(size_t const count, Function&& func) {
    if constexpr (DO_PARALLEL) {
        tbb::parallel_for(size_t{0}, count, std::forward<Function>(func));
    } else {
        for (size_t i = 0; i < count; ++i) { func(i); }
    }
}

template <typename T, bool PARALLEL = DO_PARALLEL>
void fill_array(size_t const count, T const value, T* const values) {
    if constexpr (PARALLEL) {
        for_each_iota(count, [=](auto const i) { values[i] = value; });
    } else {
        std::fill(values, values + count, value);
    }
}

template <typename T, bool PARALLEL = DO_PARALLEL>
void copy_array(size_t const count, T* const dst, T const* const src) {
    if (count < 1 || dst == src) { return; }

    if constexpr (PARALLEL) {
        for_each_iota(count, [=](auto const i) { dst[i] = src[i]; });
    } else {
        std::copy(src, src + count, dst);
    }
}

template <typename Value, typename Multiplier>
void accumulate(size_t const count,
                Multiplier const mult,
                Value* const dst,
                Value const* const src) {
    for_each_iota(count,
                  [=](auto const i) { dst[i] += mult * src[i]; });
}

template <typename Value>
void accumulate(size_t const count,
                Value* const dst,
                Value const* const src) {
    for_each_iota(count, [=](auto const i) { dst[i] += src[i]; });
}

//------------------------------------------------------------------------------
Box2f compute_bounds(size_t const particle_count, V2f const* const positions);

// origin is at 0, cell size is 1
V2i compute_grid_coord(V2f const p);

// origin is at 0, cell size given
V2i compute_grid_coord(V2f const p, float const cell_size);

// origin and cell size given
V2i compute_grid_coord(V2f const p,
                       V2f const bounds_min,
                       float const cell_size);

void compute_grid_coords(size_t const particle_count,
                         float const cell_size,
                         V2f const bounds_min,
                         V2i* const grid_coords,
                         V2f const* const positions);

void compute_z_indices(size_t const particle_count,
                       uint64_t* const z_indices,
                       V2i const* const grid_coords);

}  // namespace emerald::sph_common
