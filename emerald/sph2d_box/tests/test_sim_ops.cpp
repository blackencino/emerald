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

TEST_F(Sim_ops_test, Accumulate_gravity) {
    auto const count = positions.size();
    auto forces = positions;

    float const gravity = 9.81f;
    float const mass = 10.1235f;

    auto expected_forces = forces;
    for (auto& expected_force : expected_forces) {
        expected_force[1] -= gravity * mass;
    }

    accumulate_gravity_forces(count, mass, gravity, forces.data());

    EXPECT_EQ(expected_forces, forces);
}

}  // namespace emerald::sph2d_box