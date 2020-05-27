#pragma once

#include <emerald/sph_common/common.h>
#include <emerald/sph_common/kernels.h>

#include <emerald/util/functions.h>
#include <emerald/z_index/z_index.h>

#include <tbb/concurrent_unordered_map.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace emerald::sph_common {

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

  V2f const* const self_positions,
  V2i const* const self_grid_coords,
  V2f const* const other_positions,
  std::pair<uint64_t, size_t> const* const other_sorted_index_pairs,
  Block_map const& other_block_map,
  AcceptConditionFunction&& accept_condition_function);

void create_regular_neighborhoods(
  size_t const particle_count,
  float const max_distance,
  uint8_t* const neighbor_counts,
  Neighbor_values<size_t>* const neighbor_indices,
  Neighbor_values<float>* const neighbor_distances,
  Neighbor_values<V2f>* const neighbor_vectors_to,

  V2f const* const self_positions,
  V2i const* const self_grid_coords,
  V2f const* const other_positions,
  std::pair<uint64_t, size_t> const* const other_sorted_index_pairs,
  Block_map const& other_block_map);

void compute_neighbor_distances_and_vectors_to(
  size_t const particle_count,
  Neighbor_values<float>* const neighbor_distances,
  Neighbor_values<V2f>* const neighbor_vectors_to,
  V2f const* const self_positions,
  V2f const* const other_positions,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices);

void compute_neighbor_kernels(
  size_t const particle_count,
  float const support,
  Neighbor_values<float>* const neighbor_kernels,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_distances);

void compute_neighbor_kernel_gradients(
  size_t const particle_count,
  float const support,
  Neighbor_values<V2f>* const neighbor_kernel_gradients,
  uint8_t const* const neighbor_counts,
  Neighbor_values<V2f> const* const neighbor_vectors_to);

struct Neighborhood_pointers {
    uint8_t const* const counts = nullptr;
    Neighbor_values<size_t> const* const indices = nullptr;
    Neighbor_values<float> const* const distances = nullptr;
    Neighbor_values<V2f> const* const vectors_to = nullptr;
    Neighbor_values<float> const* const kernels = nullptr;
    Neighbor_values<V2f> const* const kernel_gradients = nullptr;
};

struct Neighborhood_vectors {
    std::vector<uint8_t> counts;
    std::vector<Neighbor_values<size_t>> indices;
    std::vector<Neighbor_values<float>> distances;
    std::vector<Neighbor_values<V2f>> vectors_to;
    std::vector<Neighbor_values<float>> kernels;
    std::vector<Neighbor_values<V2f>> kernel_gradients;

    void resize(size_t const count) {
        counts.resize(count);
        indices.resize(count);
        distances.resize(count);
        vectors_to.resize(count);
        kernels.resize(count);
        kernel_gradients.resize(count);
    }

    Neighborhood_pointers pointers() const {
        return {counts.data(),
                indices.data(),
                distances.data(),
                vectors_to.data(),
                kernels.data(),
                kernel_gradients.data()};
    }
};

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

  V2f const* const self_positions,
  V2i const* const self_grid_coords,

  V2f const* const other_positions,
  std::pair<uint64_t, size_t> const* const other_sorted_index_pairs,
  Block_map const& other_block_map,
  AcceptConditionFunction&& accept_condition_function) {
    for_each_iota(
      particle_count,
      [max_distance,
       neighbor_counts,
       neighbor_indices,
       neighbor_distances,
       neighbor_vectors_to,
       self_positions,
       self_grid_coords,
       other_positions,
       other_sorted_index_pairs,
       &other_block_map,
       accept = std::forward<AcceptConditionFunction>(
         accept_condition_function)](auto const particle_index) {
          auto const pos = self_positions[particle_index];
          auto const grid_coord = self_grid_coords[particle_index];

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
                  auto const found_iter = other_block_map.find(other_z_index);
                  if (found_iter == other_block_map.end()) { continue; }

                  // sip == sorted_index_pair
                  auto const [sip_begin, sip_end] = (*found_iter).second;
                  for (auto sip = sip_begin;
                       (nbhd_count < NEIGHBORHOOD_MAX_COUNT) &&
                       (sip != sip_end);
                       ++sip) {
                      auto const other_particle_index =
                        other_sorted_index_pairs[sip].second;
                      if (((other_positions == self_positions) &&
                           (other_particle_index == particle_index)) ||
                          !accept(particle_index, other_particle_index)) {
                          continue;
                      }

                      auto& neighbor = neighbors.at(nbhd_count);
                      neighbor.index = other_particle_index;
                      neighbor.vector_to =
                        other_positions[other_particle_index] - pos;
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

}  // namespace emerald::sph_common
