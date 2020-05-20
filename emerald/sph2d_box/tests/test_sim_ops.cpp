#include <emerald/sph2d_box/sim_ops.h>

#include <emerald/util/imath_vec_ordering.h>

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