#include <emerald/sph_common/common.h>

#include <emerald/util/imath_vec_ordering.h>
#include <emerald/z_index/z_index.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

namespace emerald::sph_common {

using namespace emerald::util;

struct Common_test : public ::testing::Test {
    Common_test() {
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

TEST_F(Common_test, Compute_bounds) {
    Box2f expected_bounds;
    for (auto const& p : positions) { expected_bounds.extendBy(p); }

    Box2f const bounds = compute_bounds(positions.size(), positions.data());
    EXPECT_EQ(expected_bounds, bounds);
    EXPECT_FALSE(bounds.isEmpty());
}

TEST_F(Common_test, Compute_grid_coord) {
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

TEST_F(Common_test, Compute_grid_coords) {
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

TEST_F(Common_test, Compute_z_indices) {
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

TEST_F(Common_test, Max_density_error) {
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

TEST_F(Common_test, Max_vector_squared_magnitude) {
    float expected = 0.0f;
    for (auto const& pos : positions) {
        expected = std::max(expected, pos.dot(pos));
    }

    auto const got =
      max_vector_squared_magnitude(positions.size(), positions.data());

    EXPECT_EQ(expected, got);
}

}  // namespace emerald::sph_common