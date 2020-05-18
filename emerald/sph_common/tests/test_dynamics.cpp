#include <emerald/sph_common/dynamics.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

namespace emerald::sph_common {

struct Dynamics_test : public ::testing::Test {
    Dynamics_test() {
        positions.resize(7927);
        velocities.resize(7927);
        forces.resize(7927);
        inv_masses.resize(7927);

        std::mt19937_64 gen{17272};
        std::uniform_real_distribution<float> pos_dist{-100.0f, 100.0f};
        for (auto& [x, y] : positions) {
            x = pos_dist(gen);
            y = pos_dist(gen);
        }

        std::uniform_real_distribution<float> vel_dist{-10.0f, 10.0f};
        for (auto& [x, y] : velocities) {
            x = vel_dist(gen);
            y = vel_dist(gen);
        }

        std::uniform_real_distribution<float> force_dist{-1.0f, 1.0f};
        for (auto& [x, y] : forces) {
            x = force_dist(gen);
            y = force_dist(gen);
        }

        std::uniform_real_distribution<float> inv_mass_dist{0.0f, 1.0e6f};
        for (auto& inv_mass : inv_masses) { inv_mass = inv_mass_dist(gen); }
    }

    std::vector<V2f> positions;
    std::vector<V2f> velocities;
    std::vector<V2f> forces;
    std::vector<float> inv_masses;
};

TEST_F(Dynamics_test, Linear_accelerations_constant_mass) {
    auto const count = positions.size();
    std::vector<V2f> linear_accelerations;
    std::vector<V2f> expected_linear_accelerations;
    float const inv_mass = 1.0f / 100.0f;

    expected_linear_accelerations.resize(count);
    std::transform(forces.begin(),
                   forces.end(),
                   expected_linear_accelerations.begin(),
                   [inv_mass](V2f const& f) { return f * inv_mass; });

    linear_accelerations.resize(count);
    compute_linear_accelerations_constant_mass(
      count, inv_mass, linear_accelerations.data(), forces.data());
    EXPECT_EQ(expected_linear_accelerations, linear_accelerations);
}

TEST_F(Dynamics_test, Linear_accelerations) {
    auto const count = positions.size();
    std::vector<V2f> linear_accelerations;
    std::vector<V2f> expected_linear_accelerations;

    expected_linear_accelerations.resize(count);
    for (size_t i = 0; i < count; ++i) {
        expected_linear_accelerations[i] = inv_masses[i] * forces[i];
    }

    linear_accelerations.resize(count);
    compute_linear_accelerations(
      count, linear_accelerations.data(), inv_masses.data(), forces.data());
    EXPECT_EQ(expected_linear_accelerations, linear_accelerations);
}

TEST_F(Dynamics_test, Forward_euler_integrate) {
    auto const count = positions.size();
    std::vector<V2f> accelerations;
    std::vector<V2f> expected_velocity_nexts;
    std::vector<V2f> velocity_nexts;
    std::vector<V2f> expected_position_nexts;
    std::vector<V2f> position_nexts;

    accelerations.resize(count);
    compute_linear_accelerations(
      count, accelerations.data(), inv_masses.data(), forces.data());

    float const dt = 1.0f / 24.0f;

    expected_velocity_nexts.resize(count);
    for (size_t i = 0; i < count; ++i) {
        expected_velocity_nexts[i] = velocities[i] + dt * accelerations[i];
    }

    velocity_nexts.resize(count);
    forward_euler_integrate_linear_values(count,
                                          dt,
                                          velocity_nexts.data(),
                                          velocities.data(),
                                          accelerations.data());
    EXPECT_EQ(expected_velocity_nexts, velocity_nexts);

    // Test in-place
    velocity_nexts = velocities;
    forward_euler_integrate_linear_values(count,
                                          dt,
                                          velocity_nexts.data(),
                                          velocity_nexts.data(),
                                          accelerations.data());
    EXPECT_EQ(expected_velocity_nexts, velocity_nexts);

    // do positions too
    expected_position_nexts.resize(count);
    for (size_t i = 0; i < count; ++i) {
        expected_position_nexts[i] =
          positions[i] + dt * expected_velocity_nexts[i];
    }

    position_nexts.resize(count);
    forward_euler_integrate_linear_values(count,
                                          dt,
                                          position_nexts.data(),
                                          positions.data(),
                                          expected_velocity_nexts.data());

    // Test in-place
    position_nexts = positions;
    forward_euler_integrate_linear_values(count,
                                          dt,
                                          position_nexts.data(),
                                          position_nexts.data(),
                                          expected_velocity_nexts.data());
}

}  // namespace emerald::sph_common