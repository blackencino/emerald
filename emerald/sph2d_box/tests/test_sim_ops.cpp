#include <emerald/sph2d_box/sim_ops.h>

#include <emerald/util/imath_vec_ordering.h>
#include <emerald/z_index/z_index.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

namespace emerald::sph2d_box {

using namespace emerald::util;

struct Sim_ops_test : public ::testing::Test {
    Sim_ops_test() {
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

TEST_F(Sim_ops_test, Compute_bounds) {
    Box2f expected_bounds;
    for (auto const& p : positions) { expected_bounds.extendBy(p); }

    Box2f const bounds = compute_bounds(positions.size(), positions.data());
    EXPECT_EQ(expected_bounds, bounds);
    EXPECT_FALSE(bounds.isEmpty());
}

TEST_F(Sim_ops_test, Compute_grid_coord) {
    V2i const expected_1{-1, 3};
    V2f const point_1{-2.4f, 9.11f};
    float const radius_1 = 2.95f;
    V2i const coord_1 = compute_grid_coord(point_1, V2f{0.0f, 0.0f}, radius_1);
    V2i const coord_1b = compute_grid_coord(point_1, radius_1);
    V2i const coord_1c = compute_grid_coord(point_1 / radius_1);

    EXPECT_EQ(expected_1, coord_1);
    EXPECT_EQ(expected_1, coord_1b);
    EXPECT_EQ(expected_1, coord_1c);

    V2i const expected_2{4, 5};
    V2f const point_2{4.1f, 5.9f};
    V2i const coord_2 = compute_grid_coord(point_2);
    EXPECT_EQ(expected_2, coord_2);

    V2i const expected_3{-5, -6};
    V2f const point_3{-4.1f, -5.9f};
    V2i const coord_3 = compute_grid_coord(point_3);
    EXPECT_EQ(expected_3, coord_3);
}

TEST_F(Sim_ops_test, Compute_grid_coords) {
    auto const count = positions.size();
    std::vector<V2i> expected_grid_coords;
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    expected_grid_coords.resize(count);
    std::transform(positions.begin(),
                   positions.end(),
                   expected_grid_coords.begin(),
                   [cell_size, bounds_min](V2f const& p) {
                       return compute_grid_coord(p, bounds_min, cell_size);
                   });

    std::vector<V2i> grid_coords;
    grid_coords.resize(count);
    compute_grid_coords(
        count, cell_size, bounds_min, grid_coords.data(), positions.data());

    EXPECT_EQ(expected_grid_coords, grid_coords);

    std::sort(grid_coords.begin(), grid_coords.end());
    grid_coords.erase(std::unique(grid_coords.begin(), grid_coords.end()),
                      grid_coords.end());
    EXPECT_LT(1000, grid_coords.size());
    EXPECT_GT(count, grid_coords.size());
}

TEST_F(Sim_ops_test, Compute_z_indices) {
    float const cell_size = 2.1f;
    V2f const bounds_min{0.0f, 0.0f};
    std::vector<V2i> grid_coords;
    grid_coords.resize(positions.size());
    compute_grid_coords(positions.size(),
                        cell_size,
                        bounds_min,
                        grid_coords.data(),
                        positions.data());

    std::sort(grid_coords.begin(), grid_coords.end());
    grid_coords.erase(std::unique(grid_coords.begin(), grid_coords.end()),
                      grid_coords.end());

    std::vector<uint64_t> expected_z_indices;
    expected_z_indices.resize(grid_coords.size());
    std::transform(grid_coords.begin(),
                   grid_coords.end(),
                   expected_z_indices.begin(),
                   [](V2i const& grid_coord) {
                       return z_index::z_index(grid_coord[0], grid_coord[1]);
                   });

    std::vector<uint64_t> z_indices;
    z_indices.resize(grid_coords.size());
    compute_z_indices(grid_coords.size(), z_indices.data(), grid_coords.data());

    EXPECT_EQ(expected_z_indices, z_indices);

    auto last = std::unique(z_indices.begin(), z_indices.end());
    EXPECT_EQ(z_indices.end(), last);
}

TEST_F(Sim_ops_test, Compute_index_pairs) {
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

TEST_F(Sim_ops_test, Sort_index_pairs) {
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

TEST_F(Sim_ops_test, Compute_block_indices) {
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

TEST_F(Sim_ops_test, Fill_blocks) {
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

TEST_F(Sim_ops_test, Create_neighborhoods) {
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

    std::vector<Neighborhood> neighborhoods;
    neighborhoods.resize(count);
    create_neighborhoods(count,
                         cell_size,
                         neighborhoods.data(),
                         positions.data(),
                         grid_coords.data(),
                         index_pairs.data(),
                         block_map);
    auto const later = std::chrono::high_resolution_clock::now();
    fmt::print(
        "Milliseconds: {}\n",
        std::chrono::duration<double, std::milli>{later - start}.count());

    // Find neighbors the hard way and make sure they're in the neighborhood
    for (size_t i = 0; i < count; ++i) {
        auto const pos_i = positions[i];
        auto const nbhd = neighborhoods[i];
        auto const nbhd_begin = nbhd.indices.data();
        auto const nbhd_end = nbhd.indices.data() + nbhd.count;
        for (size_t j = 0; j < count; ++j) {
            if (i == j) { continue; }

            auto const pos_j = positions[j];
            auto const len = (pos_i - pos_j).length();
            if (len >= cell_size) { continue; }

            // This particle is in the neighborhood.
            auto const found_iter =
                std::find(nbhd_begin, nbhd_end, static_cast<uint32_t>(j));
            if (found_iter == nbhd_end) {
                fmt::print(
                    "I: {}, J: {}, pos_i:({},{}), pos_j:({}, {}), len: {}, r: "
                    "{}\n"
                    "Grid_i: ({},{}), Grid_j: ({},{})\n",
                    i,
                    j,
                    pos_i.x,
                    pos_i.y,
                    pos_j.x,
                    pos_j.y,
                    len,
                    cell_size,
                    grid_coords[i].x,
                    grid_coords[i].y,
                    grid_coords[j].x,
                    grid_coords[j].y);
            }
            ASSERT_NE(found_iter, nbhd_end);
        }
    }

    // Loop over neighbors and make sure they're all good
    for (size_t i = 0; i < count; ++i) {
        auto const pos_i = positions[i];
        auto const& nbhd = neighborhoods[i];
        for (int ni = 0; ni < nbhd.count; ++ni) {
            auto const j = nbhd.indices[ni];
            auto const pos_j = positions[j];
            auto const len = (pos_j - pos_i).length();
            ASSERT_LT(len, cell_size);
        }
    }
}

TEST_F(Sim_ops_test, Neighborhood_equality) {
    size_t const count = 7;
    std::vector<Neighborhood> neighborhoods;
    neighborhoods.resize(count);

    auto make_neighborhood = [](std::initializer_list<int> indices) {
        Neighborhood nbhd;
        nbhd.count = static_cast<uint32_t>(indices.size());
        std::copy(indices.begin(), indices.end(), nbhd.indices.begin());
        return nbhd;
    };

    neighborhoods[0] = make_neighborhood({1, 2, 3, 4, 5});
    ASSERT_EQ(5, neighborhoods[0].count);
    ASSERT_EQ(4, neighborhoods[0].indices[3]);

    neighborhoods[1] = make_neighborhood({1, 2, 3, 4, 5});
    ASSERT_EQ(neighborhoods[0], neighborhoods[1]);

    neighborhoods[2] = make_neighborhood({5, 4, 3, 2, 1});

    neighborhoods[3] = make_neighborhood({1, 2, 3, 4, 5, 6});

    neighborhoods[4] = make_neighborhood(
        {151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96, 53,  194, 233,
         7,   225, 140, 36,  103, 30,  69,  142, 8,   99,  37, 240, 21,  10,
         174, 20,  125, 136, 171, 168, 68,  175, 74,  165, 71, 134});

    neighborhoods[5] = make_neighborhood({5,  202, 38,  147, 118, 126, 255, 82,
                                          85, 212, 207, 206, 59,  227, 47,  16,
                                          58, 17,  182, 189, 28,  42});

    neighborhoods[6] = make_neighborhood({1, 2, 3, 4, 6});

    for (size_t i = 1; i < count; ++i) {
        for (size_t j = i + 1; j < count; ++j) {
            ASSERT_NE(neighborhoods[i], neighborhoods[j]);
        }
    }
}

TEST_F(Sim_ops_test, Max_density_error) {
    std::vector<float> densities;
    constexpr float target_density = 1000.0f;
    densities.resize(7372);
    std::mt19937_64 gen{91191};
    std::uniform_real_distribution<float> dist{target_density * 0.5f,
                                               target_density * 1.5f};

    auto gen_density = [&gen, &dist]() -> float { return dist(gen); };

    std::generate(densities.begin(), densities.end(), gen_density);

    float expected_max_error = 0.0f;
    for (auto const density : densities) {
        auto const error = std::max(0.0f, density - target_density);
        expected_max_error = std::max(error, expected_max_error);
    }

    auto const max_error =
        max_density_error(densities.size(), target_density, densities.data());

    EXPECT_EQ(expected_max_error, max_error);
}

}  // namespace emerald::sph2d_box