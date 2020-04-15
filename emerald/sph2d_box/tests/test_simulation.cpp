#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace emerald::sph2d_box {

TEST(Simulation_test, Simulation_config_constructor) {
    Parameters const params;
    Simulation_config const config{params};

    fmt::print(
        "seconds per sub step: {}\n"
        "mass per particle: {}\n"
        "draw radius: {}\n"
        "pressure correction denom: {}\n",
        config.seconds_per_sub_step,
        config.mass_per_particle,
        config.draw_radius,
        config.pressure_correction_denom);
}

TEST(Simulation_test, Dam_break_initial_state) {
    Parameters const params;
    auto const state = dam_break_initial_state(params);

    fmt::print("Num created: {}\n", state.positions.size());

    EXPECT_LT(0, state.positions.size());
    EXPECT_EQ(state.positions.size(), state.velocities.size());
    EXPECT_EQ(state.positions.size(), state.colors.size());
}

}  // namespace emerald::sph2d_box