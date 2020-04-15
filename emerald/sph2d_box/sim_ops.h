#pragma once

#include <emerald/sph2d_box/foundation.h>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <array>
#include <cstdint>
#include <utility>

namespace emerald::sph2d_box {

//------------------------------------------------------------------------------
// Types
using Block_map =
    tbb::concurrent_unordered_map<uint64_t, std::pair<size_t, size_t>>;

struct Neighborhood {
    static constexpr uint32_t MAX_COUNT = 63;
    std::array<uint32_t, MAX_COUNT> indices;
    uint32_t count = 0;
};

//------------------------------------------------------------------------------
// Generics
template <typename Function>
void do_parts_work(size_t const size, Function&& func) {
    tbb::parallel_for(size_t{0}, size, std::forward<Function>(func));
}

template <typename T, bool PARALLEL = true>
void fill_array(size_t const size, T const value, T* const values) {
    if constexpr (PARALLEL) {
        do_parts_work(size, [=](auto const i) { values[i] = value; });
    } else {
        std::fill(values, values + size, value);
    }
}

template <typename T, bool PARALLEL = true>
void copy_array(size_t const size, T* const dst, T const* const src) {
    if (size < 1 || dst == src) { return; }

    if constexpr (PARALLEL) {
        do_parts_work(size, [=](auto const i) { dst[i] = src[i]; });
    } else {
        std::copy(src, src + size, dst);
    }
}

template <typename Value, typename Multiplier>
void accumulate(size_t const size,
                Multiplier const mult,
                Value* const dst,
                Value const* const src) {
    do_parts_work(size, [=](auto const i) { dst[i] += mult * src[i]; });
}

//------------------------------------------------------------------------------
// Sim ops
Box2f compute_bounds(size_t const size, V2f const* const positions);

// origin is at 0, cell size is 1
V2i compute_grid_coord(V2f const p);

// origin is at 0, cell size given
V2i compute_grid_coord(V2f const p, float const cell_size);

// origin and cell size given
V2i compute_grid_coord(V2f const p,
                       V2f const bounds_min,
                       float const cell_size);

void compute_grid_coords(size_t const size,
                         float const cell_size,
                         V2f const bounds_min,
                         V2i* const grid_coords,
                         V2f const* const positions);

void compute_z_indices(size_t const size,
                       uint64_t* const z_indices,
                       V2i const* const grid_coords);

void compute_index_pairs(size_t const size,
                         std::pair<uint64_t, size_t>* const index_pairs,
                         uint64_t const* const z_indices);

void sort_index_pairs(size_t const size,
                      std::pair<uint64_t, size_t>* const index_pairs);

// This is a weird one. It produces an array where each value is the
// ordered block index, where a block is a storage entity associated with
// each occupied grid cell. So this array will have as many unique values
// as there are unique z indices in the sorted index pairs, and they'll
// be ordered like 000011222223344444455566666677
void compute_block_indices(
    size_t const size,
    uint32_t* const block_indices,
    std::pair<uint64_t, size_t>* const sorted_index_pairs);

void fill_blocks(size_t const size,
                 std::pair<size_t, size_t>* const blocks,
                 uint32_t* const block_indices);

Block_map create_block_map(
    size_t const block_count,
    Block_map&& into,
    std::pair<size_t, size_t> const* const blocks,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs);

void create_neighborhoods(
    size_t const size,
    float const max_radius,
    Neighborhood* const neighborhoods,
    V2f const* const positions,
    V2i const* const grid_coords,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs,
    Block_map const& block_map);

void compute_external_forces(size_t const size,
                             float const mass_per_particle,
                             float const gravity,
                             V2f* const forces);

void init_pressure(size_t const size, float* const pressures);

void predict_velocities(size_t const size,
                        float const mass_per_particle,
                        float const dt,
                        V2f* const velocity_stars,
                        V2f const* const velocities,
                        V2f const* const pressure_forces,
                        V2f const* const forces);

void predict_positions(size_t const size,
                       float const dt,
                       V2f* const position_stars,
                       V2f const* const positions,
                       V2f const* const velocities);

void enforce_solid_boundaries(size_t const size,
                              float const support,
                              float const world_length,
                              V2f* const positions,
                              V2f* const velocities);

void predict_densities(size_t const size,
                       float const mass_per_particle,
                       float const support,
                       float* const densities,
                       Neighborhood const* const neighborhoods,
                       V2f const* const positions);

void update_pressures(size_t const size,
                      float const target_density,
                      float const pressure_correction_denom,
                      float* const pressures,
                      float const* const densities);

void compute_pressure_forces(size_t const size,
                             float const mass_per_particle,
                             float const support,
                             float const viscosity,
                             V2f* const pressure_forces,
                             Neighborhood const* const neighborhoods,
                             V2f const* const positions,
                             V2f const* const velocities,
                             float const* const pressures,
                             float const* const densities);

void update_velocities(size_t const size,
                       float const mass_per_particle,
                       float const dt,
                       V2f* const velocities,
                       V2f const* const pressure_forces,
                       V2f const* const forces);

void update_positions(size_t const size,
                      float const dt,
                      V2f* const positions,
                      V2f const* const velocities);

float max_density_error(size_t const size,
                        float const target_density,
                        float const* const densities);

void compute_colors(size_t const size,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities);

}  // namespace emerald::sph2d_box
