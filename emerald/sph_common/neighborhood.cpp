#include <emerald/sph_common/neighborhood.h>

#include <fmt/format.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_scan.h>
#include <tbb/parallel_sort.h>

namespace emerald::sph_common {

void compute_index_pairs(size_t const particle_count,
                         std::pair<uint64_t, size_t>* const index_pairs,
                         uint64_t const* const z_indices) {
    for_each_iota(particle_count, [=](auto const i) {
        index_pairs[i] = {z_indices[i], i};
    });
}

void sort_index_pairs(size_t const particle_count,
                      std::pair<uint64_t, size_t>* const index_pairs) {
    if (particle_count <= 1) { return; }
    if constexpr (DO_PARALLEL) {
        tbb::parallel_sort(index_pairs, index_pairs + particle_count);
    } else {
        std::sort(index_pairs, index_pairs + particle_count);
    }
}

// This is a weird one. It produces an array where each value is the
// ordered block index, where a block is a storage entity associated with
// each occupied grid cell. So this array will have as many unique values
// as there are unique z indices in the sorted index pairs, and they'll
// be ordered like 000011222223344444455566666677
void compute_block_indices(
  size_t const particle_count,
  uint32_t* const block_indices,
  std::pair<uint64_t, size_t> const* const sorted_index_pairs) {
    if (particle_count < 1) { return; }

    // Iterate starting from position 1, init the 0th entry to 0.
    block_indices[0] = 0;

    if constexpr (DO_PARALLEL) {
        // Note that we start the range at 1, so we can do i-1.
        tbb::parallel_scan(
          tbb::blocked_range<size_t>{size_t{1}, particle_count},
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
          [](uint32_t const left, uint32_t const right) {
              return left + right;
          });
    } else {
        uint32_t sum = 0;
        for (size_t i = 1; i < particle_count; ++i) {
            if (sorted_index_pairs[i - 1].first !=
                sorted_index_pairs[i].first) {
                ++sum;
            }
            block_indices[i] = sum;
        }
    }
}

void fill_blocks(size_t const particle_count,
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
    for_each_iota(particle_count, [=](auto const i) {
        auto const block_index = block_indices[i];
        if (i == 0 || block_indices[i - 1] != block_index) {
            auto& block = blocks[block_index];
            block.first = i;
            for (size_t j = i + 1; j < particle_count; ++j) {
                if (block_indices[j] != block_index) {
                    block.second = j;
                    return;
                }
            }
            // If we get here we went all the way to the end and didn't
            // find a new block, so the end is particle_count
            block.second = particle_count;
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

    for_each_iota(
      block_count, [&into, blocks, sorted_index_pairs](auto const block_i) {
          auto const block = blocks[block_i];
          auto const z_index = sorted_index_pairs[block.first].first;
          into.emplace(z_index, block);
      });

    return std::move(into);
}

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
  Block_map const& other_block_map) {
    create_neighborhoods(
      particle_count,
      max_distance,
      neighbor_counts,
      neighbor_indices,
      neighbor_distances,
      neighbor_vectors_to,
      self_positions,
      self_grid_coords,
      other_positions,
      other_sorted_index_pairs,
      other_block_map,
      [](auto const /*i*/, auto const /*j*/) { return true; });
}

void compute_neighbor_distances_and_vectors_to(
  size_t const particle_count,
  Neighbor_values<float>* const neighbor_distances,
  Neighbor_values<V2f>* const neighbor_vectors_to,
  V2f const* const self_positions,
  V2f const* const other_positions,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const position = self_positions[particle_index];
        auto const neighbor_count = neighbor_counts[particle_index];
        auto& neighbor_distance = neighbor_distances[particle_index];
        auto& neighbor_vector_to = neighbor_vectors_to[particle_index];
        auto const& nbhd_indices = neighbor_indices[particle_index];
        for (uint8_t j = 0; j < neighbor_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const delta_pos =
              other_positions[other_particle_index] - position;
            neighbor_vector_to[j] = delta_pos;
            neighbor_distance[j] = delta_pos.length();
        }
    });
}

void compute_neighbor_kernels(
  size_t const particle_count,
  float const support,
  Neighbor_values<float>* const neighbor_kernels,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_distances) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto& neighbor_kernel = neighbor_kernels[particle_index];
        auto const neighbor_count = neighbor_counts[particle_index];
        auto const& neighbor_distance = neighbor_distances[particle_index];
        for (uint8_t j = 0; j < neighbor_count; ++j) {
            neighbor_kernel[j] = kernels::W(neighbor_distance[j], support);
        }
    });
}

void compute_neighbor_kernel_gradients(
  size_t const particle_count,
  float const support,
  Neighbor_values<V2f>* const neighbor_kernel_gradients,
  uint8_t const* const neighbor_counts,
  Neighbor_values<V2f> const* const neighbor_vectors_to) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto& neighbor_kernel_gradient =
          neighbor_kernel_gradients[particle_index];
        auto const neighbor_count = neighbor_counts[particle_index];
        auto const& neighbor_vector_to = neighbor_vectors_to[particle_index];
        for (uint8_t j = 0; j < neighbor_count; ++j) {
            neighbor_kernel_gradient[j] =
              kernels::GradW(-neighbor_vector_to[j], support);
        }
    });
}

}  // namespace emerald::sph_common
