#include <emerald/sph2d_box/sim_ops.h>

#include <emerald/z_index/z_index.h>

#include <fmt/format.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_scan.h>
#include <tbb/parallel_sort.h>

#include <algorithm>
#include <cmath>

namespace emerald::sph2d_box {

Box2f compute_bounds(size_t const size, V2f const* const positions) {
    if (size < 1) { return Box2f{}; }

    return tbb::parallel_reduce(
        // Range for reduction
        tbb::blocked_range<V2f const*>{positions, positions + size},
        // Identity element
        Box2f{},
        // Reduce a subrange and partial bounds
        [](tbb::blocked_range<V2f const*> const& range, Box2f partial_bounds) {
            for (auto const& p : range) { partial_bounds.extendBy(p); }
            return partial_bounds;
        },
        // Reduce two partial bounds
        [](Box2f bounds_a, Box2f const& bounds_b) {
            bounds_a.extendBy(bounds_b);
            return bounds_a;
        });
}

V2i compute_grid_coord(V2f const p) {
    return {static_cast<int32_t>(std::floor(p[0])),
            static_cast<int32_t>(std::floor(p[1]))};
}

V2i compute_grid_coord(V2f const position, float const cell_size) {
    return compute_grid_coord(position / cell_size);
}

V2i compute_grid_coord(V2f const position,
                       V2f const bounds_min,
                       float const cell_size) {
    return compute_grid_coord(position - bounds_min, cell_size);
}

void compute_grid_coords(size_t const size,
                         float const cell_size,
                         V2f const bounds_min,
                         V2i* const grid_coords,
                         V2f const* const positions) {
    do_parts_work(size, [=](auto const i) {
        grid_coords[i] =
            compute_grid_coord(positions[i], bounds_min, cell_size);
    });
}

void compute_z_indices(size_t const size,
                       uint64_t* const z_indices,
                       V2i const* const grid_coords) {
    do_parts_work(size, [=](auto const i) {
        auto const [x, y] = grid_coords[i];
        z_indices[i] = z_index::z_index(x, y);
    });
}

void compute_index_pairs(size_t const size,
                         std::pair<uint64_t, size_t>* const index_pairs,
                         uint64_t const* const z_indices) {
    do_parts_work(size, [=](auto const i) {
        index_pairs[i] = {z_indices[i], i};
    });
}

void sort_index_pairs(size_t const size,
                      std::pair<uint64_t, size_t>* const index_pairs) {
    if (size < 1) { return; }
    tbb::parallel_sort(index_pairs, index_pairs + size);
}

// This is a weird one. It produces an array where each value is the
// ordered block index, where a block is a storage entity associated with
// each occupied grid cell. So this array will have as many unique values
// as there are unique z indices in the sorted index pairs, and they'll
// be ordered like 000011222223344444455566666677
void compute_block_indices(
    size_t const size,
    uint32_t* const block_indices,
    std::pair<uint64_t, size_t>* const sorted_index_pairs) {
    if (size < 1) { return; }

    // Iterate starting from position 1, init the 0th entry to 0.
    block_indices[0] = 0;

    // Note that we start the range at 1, so we can do i-1.
    tbb::parallel_scan(
        tbb::blocked_range<size_t>{size_t{1}, size},
        0,
        [block_indices, sorted_index_pairs](
            tbb::blocked_range<size_t> const& range,
            uint32_t const sum,
            bool const is_final_scan) {
            uint32_t temp = sum;
            for (auto i = range.begin(); i != range.end(); ++i) {
                if (sorted_index_pairs[i - 1].first !=
                    sorted_index_pairs[i].first) {
                    ++temp;
                }
                if (is_final_scan) { block_indices[i] = temp; }
            }
            return temp;
        },
        [](uint32_t const left, uint32_t const right) { return left + right; });
}

void fill_blocks(size_t const size,
                 std::pair<size_t, size_t>* const blocks,
                 uint32_t* const block_indices) {
    // This one is also weird.
    // The goal is to build each of the blocks, which is a pair
    // of indices representing the begin and end sorted index pair index
    // (ugh what a mouthful) for the particles that live in this block.
    // So what we do is iterate over the block_indices, and if we're at the
    // beginning of a block - that is, if the index of the block_index array
    // is 0, or the block index doesn't match the one before it, we iterate
    // through each position until we see a mismatch, or we reach the end.
    //
    // Most visited entries here will do nothing, but the ones that are at the
    // start of blocks will fill that whole block.
    do_parts_work(size, [=](auto const i) {
        auto const block_index = block_indices[i];
        if (i == 0 || block_indices[i - 1] != block_index) {
            auto& block = blocks[block_index];
            block.first = i;
            for (size_t j = i + 1; j < size; ++j) {
                if (block_indices[j] != block_index) {
                    block.second = j;
                    return;
                }
            }
            // If we get here we went all the way to the end and didn't
            // find a new block, so the end is size
            block.second = size;
        }
    });
}

Block_map create_block_map(
    size_t const block_count,
    Block_map&& into,
    std::pair<size_t, size_t> const* const blocks,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs) {
    into.clear();
    into.rehash(2 * block_count);

    do_parts_work(
        block_count, [&into, blocks, sorted_index_pairs](auto const block_i) {
            auto const block = blocks[block_i];
            auto const z_index = sorted_index_pairs[block.first].first;
            into.emplace(z_index, block);
        });

    return std::move(into);
}

void create_neighborhoods(
    size_t const size,
    float const max_radius,
    Neighborhood* const neighborhoods,
    V2f const* const positions,
    V2i const* const grid_coords,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs,
    Block_map const& block_map) {
    do_parts_work(size, [=](auto const particle_index) {
        auto const pos = positions[particle_index];
        auto const grid_coord = grid_coords[particle_index];
        auto& nbhd = neighborhoods[particle_index];
        nbhd.count = 0;
        for (int32_t j = grid_coord[1] - 1; j <= grid_coord[1] + 1; ++j) {
            for (int32_t i = grid_coord[0] - 1; i <= grid_coord[0] + 1; ++i) {
                auto const other_z_index = z_index::z_index(i, j);
                auto const found_iter = block_map.find(other_z_index);
                if (found_iter == block_map.end()) { continue; }

                auto const [sip_begin, sip_end] = (*found_iter).second;
                for (auto sip = sip_begin; sip != sip_end; ++sip) {
                    auto const other_particle_index =
                        sorted_index_pairs[sip].second;
                    if (other_particle_index == particle_index) { continue; }
                    auto const other_pos = positions[other_particle_index];
                    auto const other_len = (other_pos - pos).length();
                    if (other_len >= max_radius) { continue; }
                    nbhd.indices.at(nbhd.count) =
                        static_cast<uint32_t>(other_particle_index);
                    ++nbhd.count;
                    if (nbhd.count == Neighborhood::MAX_COUNT) {
                        std::sort(nbhd.indices.data(),
                                  nbhd.indices.data() + nbhd.count);
                        return;
                    }
                }
            }
        }
        std::sort(nbhd.indices.data(), nbhd.indices.data() + nbhd.count);
    });
}

// Block_map create_blocks(size_t const size,
//                         Block_map&& into,
//                         Index_pair const* const index_pairs) {
//     into.clear();
//     into.rehash(size / 2);
//     do_parts_work(
//         size, [size, block_map = std::move(into), index_pairs](auto const i)
//         {
//             auto const [z_index, part_index] = index_pairs[i];
//             auto& block = block_map[z_index];
//             block = extend_by(block, static_cast<uint32_t>(part_index));
//         });
//     return std::move(into);
// }

// struct Neighborhood {
//     std::array<uint32_t, 63> neighbor_indices;
//     uint32_t count = 0;
// };

// void find_neighborhoods(size_t const size,
//                         Neighborhood* const neighborhoods,
//                         Block_map const& block_map,

//                         ) {

//     do_parts_work(size, [&](auto const p) {
//         auto const [grid_i, grid_j] = grid_coords[p];
//         for (uint32_t j = _decr(grid_j); j < grid_j+1; ++j) {
//             for (uint32_t i = _decr(grid_i); i < grid_i+1; ++i) {
//                 auto const z_index_ij = z_index(i, j);

//                 auto const found_iter = block_map.find(z_index_ij);
//                 if (found_iter == block_map.end()) {
//                     continue;
//                 }

//                 auto const block = (*found_iter);

//                 for (int sorted_

//             }
//         }
//     for ( IntT p = i_begin; p < i_end; ++p )
//     {
//         // Get the grid location.
//         const V2UIT &grid = m_particles.grid[p];
//         const V2T &Pself = m_particles.positionStar[p];
//         FloatT &densityStar = m_particles.densityStar[p];

//         FloatT muchnessPredict = 0.0;

//         // Loop over 9 cells.
//         for ( UintT j = _decr( grid.y ); j <= grid.y+1; ++j )
//         {
//             for ( UintT i = _decr( grid.x ); i <= grid.x+1; ++i )
//             {
//                 // Get the z-index of this block.
//                 UintT zIndex = m_zIndex[ V2UIT( i, j ) ];

//                 // Try to find this block.
//                 // If we can't find it, continue!
//                 BlockHashMap::const_accessor a;
//                 if ( !m_blocks->find( a, zIndex ) )
//                 {
//                     a.release();
//                     continue;
//                 }

//                 // Get the block out the accessor and release it.
//                 Block b = a->second;
//                 a.release();

//                 // Block has a range of particle indices
//                 // in it. They refer to the sorted indices.
//                 for ( IntT indexPairsI = b.indexPairsBegin;
//                       indexPairsI != b.indexPairsEnd; ++indexPairsI )
//                 {
//                     IntT otherP =
//                         m_particles.indexPairs[indexPairsI].index;

//                     // We don't skip self here.
//                     if ( otherP == p )
//                     {
//                         muchnessPredict += Kernels::W( 0.0, H );
//                     }
//                     else
//                     {
//                         const V2T &Pother =
//                             m_particles.positionStar[otherP];

//                         V2T dP = Pself - Pother;
//                         FloatT dpL = dP.length();
//                         if ( dpL >= 2.0*H )
//                         {
//                             continue;
//                         }
//                         else
//                         {
//                             muchnessPredict += Kernels::W( dpL, H );
//                         }
//                     }
//                 }
//             }
//         }

// }

}  // namespace emerald::sph2d_box
