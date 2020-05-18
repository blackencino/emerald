#pragma once

#include <emerald/sph2d_box/foundation.h>
#include <emerald/sph2d_box/tag.h>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <array>
#include <cstdint>
#include <unordered_map>
#include <utility>

namespace emerald::sph2d_box {

constexpr bool DO_PARALLEL = true;

//------------------------------------------------------------------------------
// Types
template <bool>
struct Block_map_helper;

template <>
struct Block_map_helper<true> {
    using map =
      tbb::concurrent_unordered_map<uint64_t, std::pair<size_t, size_t>>;
};

template <>
struct Block_map_helper<false> {
    using map = std::unordered_map<uint64_t, std::pair<size_t, size_t>>;
};

using Block_map = Block_map_helper<DO_PARALLEL>::map;

static constexpr uint8_t NEIGHBORHOOD_MAX_COUNT = 64;

template <typename T>
using Neighbor_values = std::array<T, NEIGHBORHOOD_MAX_COUNT>;

//------------------------------------------------------------------------------
// Generics
template <typename Function>
void for_each_iota(size_t const particle_count, Function&& func) {
    if constexpr (DO_PARALLEL) {
        tbb::parallel_for(size_t{0}, size, std::forward<Function>(func));
    } else {
        for (size_t i = 0; i < size; ++i) { func(i); }
    }
}

template <typename T, bool PARALLEL = DO_PARALLEL>
void fill_array(size_t const particle_count, T const value, T* const values) {
    if constexpr (PARALLEL) {
        for_each_iota(size, [=](auto const i) { values[i] = value; });
    } else {
        std::fill(values, values + size, value);
    }
}

template <typename T, bool PARALLEL = DO_PARALLEL>
void copy_array(size_t const particle_count, T* const dst, T const* const src) {
    if (size < 1 || dst == src) { return; }

    if constexpr (PARALLEL) {
        for_each_iota(size, [=](auto const i) { dst[i] = src[i]; });
    } else {
        std::copy(src, src + size, dst);
    }
}

template <typename Value, typename Multiplier>
void accumulate(size_t const particle_count,
                Multiplier const mult,
                Value* const dst,
                Value const* const src) {
    for_each_iota(size, [=](auto const i) { dst[i] += mult * src[i]; });
}

//------------------------------------------------------------------------------
// Sim ops
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

void compute_index_pairs(size_t const particle_count,
                         std::pair<uint64_t, size_t>* const index_pairs,
                         uint64_t const* const z_indices);

void sort_index_pairs(size_t const particle_count,
                      std::pair<uint64_t, size_t>* const index_pairs);

// This is a weird one. It produces an array where each value is the
// ordered block index, where a block is a storage entity associated with
// each occupied grid cell. So this array will have as many unique values
// as there are unique z indices in the sorted index pairs, and they'll
// be ordered like 000011222223344444455566666677
void compute_block_indices(
  size_t const particle_count,
  uint32_t* const block_indices,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs);

void fill_blocks(size_t const particle_count,
                 std::pair<size_t, size_t>* const blocks,
                 uint32_t* const block_indices);

Block_map create_block_map(
  size_t const block_count,
  Block_map&& into,
  std::pair<size_t, size_t> const* const blocks,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs);



// Pass in an accept function that takes two indices and determines whether the
// particle at the second index can be a neighbor (of this type) of the particle
// at the first index
template <typename AcceptConditionFunction>
void create_neighborhoods(
  size_t const particle_count,
  float const max_distance,
  uint8_t* const neighbor_counts,
  Neighbor_values<size_t>* const neighbor_indices,
  Neighbor_values<float>* const neighbor_distances,
  Neighbor_values<V2f>* const neighbor_vectors_to,

  V2f const* const positions,
  V2i const* const grid_coords,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs,
  Block_map const& block_map,
  AcceptConditionFunction&& accept_condition_function);



void create_neighborhoods(
  size_t const particle_count,
  float const max_radius,
  Neighborhood* const neighborhoods,
  V2f const* const positions,
  V2i const* const grid_coords,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs,
  Block_map const& block_map);

void compute_external_forces(size_t const particle_count,
                             float const mass_per_particle,
                             float const gravity,
                             V2f* const forces,
                             V2f const* const positions,
                             V2f const* const velocities);

void init_pressure(size_t const particle_count, float* const pressures);

void predict_velocities(size_t const particle_count,
                        float const mass_per_particle,
                        float const dt,
                        V2f* const velocity_stars,
                        V2f const* const velocities,
                        V2f const* const pressure_forces,
                        V2f const* const forces);

void predict_positions(size_t const particle_count,
                       float const dt,
                       V2f* const position_stars,
                       V2f const* const positions,
                       V2f const* const velocities);

void reset_tags(size_t const particle_count, Tag* const tags);

void identify_solid_boundaries_and_correct_pressure_forces(
  size_t const particle_count,
  float const support,
  float const world_length,
  float const mass_per_particle,
  Tag* const tags,
  V2f* const pressure_forces,
  V2f const* const positions);

void enforce_solid_boundaries(size_t const particle_count,
                              float const support,
                              float const world_length,
                              V2f* const positions,
                              V2f* const velocities);

void predict_densities(size_t const particle_count,
                       float const mass_per_particle,
                       float const support,
                       float* const densities,
                       Neighborhood const* const neighborhoods,
                       V2f const* const positions);

void update_pressures(size_t const particle_count,
                      float const target_density,
                      float const pressure_correction_denom,
                      float* const pressures,
                      float const* const densities);

void compute_pressure_forces(size_t const particle_count,
                             float const mass_per_particle,
                             float const support,
                             float const viscosity,
                             V2f* const pressure_forces,
                             Neighborhood const* const neighborhoods,
                             V2f const* const positions,
                             V2f const* const velocities,
                             float const* const pressures,
                             float const* const densities);

void update_velocities(size_t const particle_count,
                       float const mass_per_particle,
                       float const dt,
                       V2f* const velocities,
                       V2f const* const pressure_forces,
                       V2f const* const forces);

void update_positions(size_t const particle_count,
                      float const dt,
                      V2f* const positions,
                      V2f const* const velocities);

float max_density_error(size_t const particle_count,
                        float const target_density,
                        float const* const densities);

void compute_colors(size_t const particle_count,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities);


//------------------------------------------------------------------------------
// TEMPLATE IMPLEMENTATIONS
//------------------------------------------------------------------------------


struct Neighbor {
    size_t index = 0;
    float distance = 0.0f;
    V2f vector_to = V2f{0.0f, 0.0f};
};

inline bool neighbor_order(Neighbor const& a, Neighbor const& b) {
    if (a.distance < b.distance) {
        return true;
    } else if (a.distance > b.distance) {
        return false;
    } else {
        return a.index < b.index;
    }
}

using Neighbors = std::array<Neighbor, NEIGHBORHOOD_MAX_COUNT>;

inline void sort_neighbors_in_place(Neighbors& neighbors, uint8_t const count) {
    std::sort(neighbors.begin(), neighbors.begin() + count, neighbor_order);
}

//------------------------------------------------------------------------------
// Pass in an accept function that takes two indices and determines whether the
// particle at the second index can be a neighbor (of this type) of the particle
// at the first index
template <typename AcceptConditionFunction>
void create_neighborhoods(
  size_t const particle_count,
  float const max_distance,
  uint8_t* const neighbor_counts,
  Neighbor_values<size_t>* const neighbor_indices,
  Neighbor_values<float>* const neighbor_distances,
  Neighbor_values<V2f>* const neighbor_vectors_to,

  V2f const* const positions,
  V2i const* const grid_coords,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs,
  Block_map const& block_map,
  AcceptConditionFunction&& accept_condition_function) {
    for_each_iota(
      particle_count,
      [max_distance,
       neighbor_counts,
       neighbor_indices,
       neighbor_distances,
       neighbor_vectors_to,
       positions,
       grid_coords,
       sorted_index_pairs,
       &block_map,
       accept = std::forward<AcceptConditionFunction>(
         accept_condition_function)](auto const particle_index) {
          auto const pos = positions[particle_index];
          auto const grid_coord = grid_coords[particle_index];

          Neighbors neighbors;
          uint8_t nbhd_count = 0;
          for (int32_t j = grid_coord[1] - 1;
               (nbhd_count < NEIGHBORHOOD_MAX_COUNT) &&
               (j <= grid_coord[1] + 1);
               ++j) {
              for (int32_t i = grid_coord[0] - 1;
                   (nbhd_count < NEIGHBORHOOD_MAX_COUNT) &&
                   (i <= grid_coord[0] + 1);
                   ++i) {
                  auto const other_z_index = z_index::z_index(i, j);
                  auto const found_iter = block_map.find(other_z_index);
                  if (found_iter == block_map.end()) { continue; }

                  // sip == sorted_index_pair
                  auto const [sip_begin, sip_end] = (*found_iter).second;
                  for (auto sip = sip_begin; sip != sip_end; ++sip) {
                      auto const other_particle_index =
                        sorted_index_pairs[sip].second;
                      if ((other_particle_index == particle_index) ||
                          !accept(particle_index, other_particle_index)) {
                          continue;
                      }

                      auto& neighbor = neighbors.at(nbhd_count);
                      neighbor.index = other_particle_index;
                      neighbor.vector_to =
                        positions[other_particle_index] - pos;
                      neighbor.distance = neighbor.vector_to.length();

                      if (neighbor.distance >= max_distance) { continue; }
                      ++nbhd_count;
                  }
              }
          }
          sort_neighbors_in_place(neighbors, nbhd_count);
          neighbor_counts[particle_index] = nbhd_count;
          auto& nbr_indices = neighbor_indices[particle_index];
          auto& nbr_distances = neighbor_distances[particle_index];
          auto& nbr_vectors_to = neighbor_vectors_to[particle_index];
          for (uint8_t neighbor_i = 0; neighbor_i < nbhd_count; ++neighbor_i) {
              auto const& neighbor = neighbors[neighbor_i];
              nbr_indices[neighbor_i] = neighbor.index;
              nbr_distances[neighbor_i] = neighbor.distance;
              nbr_vectors_to[neighbor_i] = neighbor.vector_to;
          }
      });
}

}  // namespace emerald::sph2d_box
