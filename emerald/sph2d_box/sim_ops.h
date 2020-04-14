#pragma once

#include <emerald/sph2d_box/foundation.h>

#include <tbb/parallel_for.h>
#include <tbb/concurrent_unordered_map.h>

#include <cstdint>
#include <utility>
#include <array>

namespace emerald::sph2d_box {

template <typename Function>
void do_parts_work(size_t const size, Function&& func) {
    tbb::parallel_for(size_t{0}, size, std::forward<Function>(func));
}

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

using Block_map = tbb::concurrent_unordered_map<uint64_t, std::pair<size_t, size_t>>;

Block_map create_block_map(
    size_t const block_count,
    Block_map&& into,
    std::pair<size_t, size_t> const* const blocks,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs);

struct Neighborhood {
    static constexpr uint32_t MAX_COUNT = 63;
    std::array<uint32_t, MAX_COUNT> indices;
    uint32_t count = 0;
};

void create_neighborhoods(size_t const size,
                          float const max_radius,
                          Neighborhood* const neighborhoods,
                          V2f const* const positions,
                          V2i const* const grid_coords,
                          std::pair<uint64_t, size_t> const* const sorted_index_pairs,
                          Block_map const& block_map);

}  // namespace emerald::sph2d_box
