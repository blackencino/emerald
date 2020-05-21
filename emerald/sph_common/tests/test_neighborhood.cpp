#include <emerald/sph_common/neighborhood.h>

#include <emerald/util/format.h>
#include <emerald/util/imath_vec_ordering.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

namespace emerald::sph_common {

using namespace emerald::util;

struct Neighborhood_test : public ::testing::Test {
    Neighborhood_test() {
        positions.resize(7927);

        std::mt19937_64 gen{17272};
        std::uniform_real_distribution<float> pos_dist{-100.0f, 100.0f};
        for (auto& [x, y] : positions) {
            x = pos_dist(gen);
            y = pos_dist(gen);
        }
    }

    std::vector<V2f> positions;
};

TEST_F(Neighborhood_test, Compute_index_pairs) {
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(positions.size());
    compute_grid_coords(positions.size(),
                        cell_size,
                        bounds_min,
                        grid_coords.data(),
                        positions.data());

    std::vector<uint64_t> z_indices;
    z_indices.resize(grid_coords.size());
    compute_z_indices(grid_coords.size(), z_indices.data(), grid_coords.data());

    std::vector<std::pair<uint64_t, size_t>> expected_index_pairs;
    expected_index_pairs.resize(grid_coords.size());
    for (size_t i = 0; i < z_indices.size(); ++i) {
        expected_index_pairs[i] = {z_indices[i], i};
    }

    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    index_pairs.resize(grid_coords.size());
    compute_index_pairs(
      grid_coords.size(), index_pairs.data(), z_indices.data());

    EXPECT_EQ(expected_index_pairs, index_pairs);
}

TEST_F(Neighborhood_test, Sort_index_pairs) {
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(positions.size());
    compute_grid_coords(positions.size(),
                        cell_size,
                        bounds_min,
                        grid_coords.data(),
                        positions.data());

    std::vector<uint64_t> z_indices;
    z_indices.resize(grid_coords.size());
    compute_z_indices(grid_coords.size(), z_indices.data(), grid_coords.data());

    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    index_pairs.resize(grid_coords.size());
    compute_index_pairs(
      grid_coords.size(), index_pairs.data(), z_indices.data());

    std::vector<std::pair<uint64_t, size_t>> expected_sorted = index_pairs;
    std::sort(expected_sorted.begin(), expected_sorted.end());

    sort_index_pairs(index_pairs.size(), index_pairs.data());

    EXPECT_EQ(expected_sorted, index_pairs);
}

TEST_F(Neighborhood_test, Compute_block_indices) {
    size_t const count = positions.size();
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(count);
    compute_grid_coords(
      count, cell_size, bounds_min, grid_coords.data(), positions.data());

    std::vector<uint64_t> z_indices;
    z_indices.resize(count);
    compute_z_indices(count, z_indices.data(), grid_coords.data());

    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    index_pairs.resize(count);
    compute_index_pairs(count, index_pairs.data(), z_indices.data());
    sort_index_pairs(count, index_pairs.data());

    std::vector<uint32_t> block_indices;
    block_indices.resize(count);
    compute_block_indices(count, block_indices.data(), index_pairs.data());

    std::vector<uint32_t> expected_block_indices;
    expected_block_indices.resize(count);
    expected_block_indices[0] = 0;
    uint32_t block_index = 0;
    for (size_t i = 1; i < count; ++i) {
        if (index_pairs[i - 1].first != index_pairs[i].first) { ++block_index; }
        expected_block_indices[i] = block_index;
    }
    size_t const expected_block_count = 1 + block_index;

    // The number of blocks should be the same as the number of unique
    // z indices. So we sort them and remove the non-unique ones.
    std::sort(z_indices.begin(), z_indices.end());
    z_indices.erase(std::unique(z_indices.begin(), z_indices.end()),
                    z_indices.end());

    EXPECT_EQ(expected_block_count, z_indices.size());
    EXPECT_EQ(block_index, block_indices.back());
    EXPECT_EQ(expected_block_indices, block_indices);
}

TEST_F(Neighborhood_test, Fill_blocks) {
    size_t const count = positions.size();
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(count);
    compute_grid_coords(
      count, cell_size, bounds_min, grid_coords.data(), positions.data());

    std::vector<uint64_t> z_indices;
    z_indices.resize(count);
    compute_z_indices(count, z_indices.data(), grid_coords.data());

    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    index_pairs.resize(count);
    compute_index_pairs(count, index_pairs.data(), z_indices.data());
    sort_index_pairs(count, index_pairs.data());

    std::vector<uint32_t> block_indices;
    block_indices.resize(count);
    compute_block_indices(count, block_indices.data(), index_pairs.data());

    size_t const block_count = block_indices.back() + 1;
    EXPECT_GT(block_count, 1000);
    std::vector<std::pair<size_t, size_t>> blocks;
    blocks.resize(block_count);
    fill_blocks(count, blocks.data(), block_indices.data());

    for (size_t i = 0; i < count; ++i) {
        auto const block_index = block_indices[i];
        auto const& block = blocks[block_index];
        EXPECT_GE(i, block.first);
        EXPECT_LT(i, block.second);
    }

    for (size_t i = 0; i < block_count; ++i) {
        auto const& block = blocks[i];
        for (auto j = block.first; j != block.second; ++j) {
            EXPECT_LT(j, count);
            EXPECT_EQ(block_indices[j], i);
        }
    }
}

TEST_F(Neighborhood_test, Create_regular_neighborhoods) {
    auto const start = std::chrono::high_resolution_clock::now();
    size_t const count = positions.size();
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(count);
    compute_grid_coords(
      count, cell_size, bounds_min, grid_coords.data(), positions.data());

    std::vector<uint64_t> z_indices;
    z_indices.resize(count);
    compute_z_indices(count, z_indices.data(), grid_coords.data());

    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    index_pairs.resize(count);
    compute_index_pairs(count, index_pairs.data(), z_indices.data());
    sort_index_pairs(count, index_pairs.data());

    std::vector<uint32_t> block_indices;
    block_indices.resize(count);
    compute_block_indices(count, block_indices.data(), index_pairs.data());

    size_t const block_count = block_indices.back() + 1;
    std::vector<std::pair<size_t, size_t>> blocks;
    blocks.resize(block_count);
    fill_blocks(count, blocks.data(), block_indices.data());

    Block_map block_map{block_count * 2};
    block_map = create_block_map(
      block_count, std::move(block_map), blocks.data(), index_pairs.data());

    std::vector<uint8_t> neighbor_counts;
    std::vector<Neighbor_values<size_t>> neighbor_indices;
    std::vector<Neighbor_values<float>> neighbor_distances;
    std::vector<Neighbor_values<V2f>> neighbor_vectors_to;
    neighbor_counts.resize(count);
    neighbor_indices.resize(count);
    neighbor_distances.resize(count);
    neighbor_vectors_to.resize(count);
    create_regular_neighborhoods(count,
                                 cell_size,
                                 neighbor_counts.data(),
                                 neighbor_indices.data(),
                                 neighbor_distances.data(),
                                 neighbor_vectors_to.data(),
                                 positions.data(),
                                 grid_coords.data(),
                                 positions.data(),
                                 index_pairs.data(),
                                 block_map);
    auto const later = std::chrono::high_resolution_clock::now();
    fmt::print(
      "Milliseconds: {}\n",
      std::chrono::duration<double, std::milli>{later - start}.count());

    // Find neighbors the hard way and make sure they're in the neighborhood
    for (size_t i = 0; i < count; ++i) {
        auto const pos_i = positions[i];
        auto const nbhd_count = neighbor_counts[i];
        auto const& nbhd_indices = neighbor_indices[i];
        auto const& nbhd_distances = neighbor_distances[i];
        auto const& nbhd_vectors_to = neighbor_vectors_to[i];
        auto const nbhd_begin = nbhd_indices.data();
        auto const nbhd_end = nbhd_indices.data() + nbhd_count;

        uint8_t found_neighbors = 0;
        for (size_t j = 0; j < count; ++j) {
            if (i == j) { continue; }

            auto const pos_j = positions[j];
            auto const vector_to = pos_j - pos_i;
            auto const distance = vector_to.length();
            if (distance >= cell_size) { continue; }

            ++found_neighbors;

            // This particle is in the neighborhood.
            auto const found_iter = std::find(nbhd_begin, nbhd_end, j);
            if (found_iter == nbhd_end) {
                fmt::print(
                  "I: {}, J: {}, pos_i: {}, pos_j: {}, len: {}, r: "
                  "{}\n"
                  "Grid_i: {}, Grid_j: {}\n",
                  i,
                  j,
                  pos_i,
                  pos_j,
                  distance,
                  cell_size,
                  grid_coords[i],
                  grid_coords[j]);
            }
            ASSERT_NE(found_iter, nbhd_end);

            auto const offset = std::distance(nbhd_begin, found_iter);

            ASSERT_EQ(vector_to, nbhd_vectors_to[offset]);
            ASSERT_EQ(distance, nbhd_distances[offset]);
        }
        ASSERT_EQ(found_neighbors, nbhd_count);
    }

    // Loop over neighbors and make sure they're all good
    for (size_t i = 0; i < count; ++i) {
        auto const pos_i = positions[i];
        auto const nbhd_count = neighbor_counts[i];
        auto const& nbhd_indices = neighbor_indices[i];
        for (int ni = 0; ni < nbhd_count; ++ni) {
            auto const j = nbhd_indices[ni];
            auto const pos_j = positions[j];
            auto const len = (pos_j - pos_i).length();
            ASSERT_LT(len, cell_size);
        }
    }
}

TEST_F(Neighborhood_test, Compute_neighbor_distances_and_vectors_to) {
    size_t const count = positions.size();
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(count);
    compute_grid_coords(
      count, cell_size, bounds_min, grid_coords.data(), positions.data());

    std::vector<uint64_t> z_indices;
    z_indices.resize(count);
    compute_z_indices(count, z_indices.data(), grid_coords.data());

    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    index_pairs.resize(count);
    compute_index_pairs(count, index_pairs.data(), z_indices.data());
    sort_index_pairs(count, index_pairs.data());

    std::vector<uint32_t> block_indices;
    block_indices.resize(count);
    compute_block_indices(count, block_indices.data(), index_pairs.data());

    size_t const block_count = block_indices.back() + 1;
    std::vector<std::pair<size_t, size_t>> blocks;
    blocks.resize(block_count);
    fill_blocks(count, blocks.data(), block_indices.data());

    Block_map block_map{block_count * 2};
    block_map = create_block_map(
      block_count, std::move(block_map), blocks.data(), index_pairs.data());

    std::vector<uint8_t> neighbor_counts;
    std::vector<Neighbor_values<size_t>> neighbor_indices;
    std::vector<Neighbor_values<float>> expected_neighbor_distances;
    std::vector<Neighbor_values<V2f>> expected_neighbor_vectors_to;
    neighbor_counts.resize(count);
    neighbor_indices.resize(count);
    expected_neighbor_distances.resize(count);
    expected_neighbor_vectors_to.resize(count);
    create_regular_neighborhoods(count,
                                 cell_size,
                                 neighbor_counts.data(),
                                 neighbor_indices.data(),
                                 expected_neighbor_distances.data(),
                                 expected_neighbor_vectors_to.data(),
                                 positions.data(),
                                 grid_coords.data(),
                                 positions.data(),
                                 index_pairs.data(),
                                 block_map);

    std::vector<Neighbor_values<float>> neighbor_distances;
    std::vector<Neighbor_values<V2f>> neighbor_vectors_to;
    neighbor_distances.resize(count);
    neighbor_vectors_to.resize(count);
    compute_neighbor_distances_and_vectors_to(count,
                                              neighbor_distances.data(),
                                              neighbor_vectors_to.data(),
                                              positions.data(),
                                              positions.data(),
                                              neighbor_counts.data(),
                                              neighbor_indices.data());

    for (size_t i = 0; i < count; ++i) {
        auto const nbhd_count = neighbor_counts[i];
        if (!nbhd_count) { continue; }

        auto const& expected_nbhd_distances = expected_neighbor_distances[i];
        auto const& expected_nbhd_vectors_to = expected_neighbor_vectors_to[i];
        auto const& nbhd_distances = neighbor_distances[i];
        auto const& nbhd_vectors_to = neighbor_vectors_to[i];

        ASSERT_TRUE(std::equal(expected_nbhd_distances.begin(),
                               expected_nbhd_distances.begin() + nbhd_count,
                               nbhd_distances.begin()));
        ASSERT_TRUE(std::equal(expected_nbhd_vectors_to.begin(),
                               expected_nbhd_vectors_to.begin() + nbhd_count,
                               nbhd_vectors_to.begin()));
    }
}

}  // namespace emerald::sph_common