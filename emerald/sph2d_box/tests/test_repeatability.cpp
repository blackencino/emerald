#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/initial_state.h>
#include <emerald/sph2d_box/neighborhoods_from_state.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/solids.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/neighborhood.h>
#include <emerald/util/format.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace emerald::sph2d_box {

using namespace emerald::util;

constexpr int REPEATABILITY_TEST_COUNT = 100;

struct Repeatability_test : public ::testing::Test {
    Parameters params;
    Simulation_config config;
    Solid_state solid_state;
    State state;

    Repeatability_test()
      : params()
      , config(params)
      , solid_state(world_walls_initial_solid_state(params))
      , state(dam_break_initial_state(params, solid_state)) {
    }
};

TEST_F(Repeatability_test, Simulation_config_constructor) {
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

TEST_F(Repeatability_test, Dam_break_initial_state) {
    fmt::print("Num created: {}\n", state.positions.size());

    EXPECT_LT(0, state.positions.size());
    EXPECT_EQ(state.positions.size(), state.velocities.size());
    EXPECT_EQ(state.positions.size(), state.colors.size());
}

TEST_F(Repeatability_test, Grid_coords_repeatability) {
    auto const count = state.positions.size();
    auto const cell_size = config.params.support * 2.0f;

    Temp_data temp;
    compute_all_neighbhorhoods(config, state, solid_state, temp);

    Temp_data temp2;
    temp2.grid_coords.resize(count);
    for (int i = 0; i < REPEATABILITY_TEST_COUNT; ++i) {
        compute_grid_coords(count,
                            cell_size,
                            V2f{0.0f, 0.0f},
                            temp2.grid_coords.data(),
                            state.positions.data());
        ASSERT_EQ(temp.grid_coords, temp2.grid_coords);

        temp2.z_indices.resize(count);
        compute_z_indices(
          count, temp2.z_indices.data(), temp2.grid_coords.data());
        ASSERT_EQ(temp.z_indices, temp2.z_indices);

        temp2.index_pairs.resize(count);
        compute_index_pairs(
          count, temp2.index_pairs.data(), temp2.z_indices.data());
        // ASSERT_EQ(temp.index_pairs, temp2.index_pairs);

        sort_index_pairs(count, temp2.index_pairs.data());
        ASSERT_EQ(temp.index_pairs, temp2.index_pairs);

        temp2.block_indices.resize(count);
        compute_block_indices(
          count, temp2.block_indices.data(), temp2.index_pairs.data());
        ASSERT_EQ(temp.block_indices, temp2.block_indices);

        size_t const block_count = temp2.block_indices.back() + 1;
        temp2.blocks.resize(block_count);
        fill_blocks(count, temp2.blocks.data(), temp2.block_indices.data());
        ASSERT_EQ(temp.blocks, temp2.blocks);

        temp2.block_map = create_block_map(block_count,
                                           std::move(temp2.block_map),
                                           temp2.blocks.data(),
                                           temp2.index_pairs.data());
        temp2.neighborhood.resize(count);
        create_regular_neighborhoods(count,
                                     cell_size,
                                     temp2.neighborhood.counts.data(),
                                     temp2.neighborhood.indices.data(),
                                     temp2.neighborhood.distances.data(),
                                     temp2.neighborhood.vectors_to.data(),
                                     state.positions.data(),
                                     temp2.grid_coords.data(),
                                     state.positions.data(),
                                     temp2.index_pairs.data(),
                                     temp2.block_map);

        ASSERT_EQ(temp.neighborhood.indices, temp2.neighborhood.indices);
    }
}

#if 0
TEST_F(Repeatability_test, Neighborhood_repeatability) {
    Temp_data temp;
    compute_all_neighbhorhoods(config, state, temp);

    Temp_data temp2;
    for (int i = 0; i < REPEATABILITY_TEST_COUNT; ++i) {
        compute_all_neighbhorhoods(config, state, temp2);
        ASSERT_EQ(temp.neighborhoods, temp2.neighborhoods);
    }
}

TEST_F(Repeatability_test, Density_repeatability) {
    auto const count = state.positions.size();
    Temp_data temp;
    temp.forces.resize(
        count, V2f{0.0f, -config.mass_per_particle * config.params.gravity});
    temp.pressure_forces.resize(count, V2f{0.0f, 0.0f});

    for (int tick = 0; tick < 3; ++tick) {
        update_velocities(count,
                          config.mass_per_particle,
                          config.seconds_per_sub_step,
                          state.velocities.data(),
                          temp.pressure_forces.data(),
                          temp.forces.data());

        update_positions(count,
                         config.seconds_per_sub_step,
                         state.positions.data(),
                         state.velocities.data());
    }

    compute_all_neighbhorhoods(config, state, temp);

    temp.densities.resize(count);
    predict_densities(count,
                      config.mass_per_particle,
                      config.params.support,
                      temp.densities.data(),
                      temp.neighborhoods.data(),
                      state.positions.data());

    std::vector<float> densities2;
    densities2.resize(count);
    for (int i = 0; i < REPEATABILITY_TEST_COUNT; ++i) {
        predict_densities(count,
                          config.mass_per_particle,
                          config.params.support,
                          densities2.data(),
                          temp.neighborhoods.data(),
                          state.positions.data());

        ASSERT_EQ(temp.densities, densities2);
    }
}

TEST_F(Repeatability_test, Pressure_repeatability) {
    auto const count = state.positions.size();
    Temp_data temp;
    temp.forces.resize(
        count, V2f{0.0f, -config.mass_per_particle * config.params.gravity});
    temp.pressure_forces.resize(count, V2f{0.0f, 0.0f});

    for (int tick = 0; tick < 3; ++tick) {
        update_velocities(count,
                          config.mass_per_particle,
                          config.seconds_per_sub_step,
                          state.velocities.data(),
                          temp.pressure_forces.data(),
                          temp.forces.data());

        update_positions(count,
                         config.seconds_per_sub_step,
                         state.positions.data(),
                         state.velocities.data());
    }
    compute_all_neighbhorhoods(config, state, temp);

    temp.densities.resize(count);
    predict_densities(count,
                      config.mass_per_particle,
                      config.params.support,
                      temp.densities.data(),
                      temp.neighborhoods.data(),
                      state.positions.data());

    temp.pressures.resize(count, 0.0f);
    init_pressure(count, temp.pressures.data());

    update_pressures(count,
                     config.params.target_density,
                     config.pressure_correction_denom,
                     temp.pressures.data(),
                     temp.densities.data());

    std::vector<float> pressures2;
    pressures2.resize(count);
    for (int i = 0; i < REPEATABILITY_TEST_COUNT; ++i) {
        init_pressure(count, pressures2.data());
        update_pressures(count,
                         config.params.target_density,
                         config.pressure_correction_denom,
                         pressures2.data(),
                         temp.densities.data());

        ASSERT_EQ(temp.pressures, pressures2);
    }
}

TEST_F(Repeatability_test, Pressure_force_repeatability) {
    auto const count = state.positions.size();
    Temp_data temp;
    temp.forces.resize(
        count, V2f{0.0f, -config.mass_per_particle * config.params.gravity});
    temp.pressure_forces.resize(count, V2f{0.0f, 0.0f});

    for (int tick = 0; tick < 3; ++tick) {
        update_velocities(count,
                          config.mass_per_particle,
                          config.seconds_per_sub_step,
                          state.velocities.data(),
                          temp.pressure_forces.data(),
                          temp.forces.data());

        update_positions(count,
                         config.seconds_per_sub_step,
                         state.positions.data(),
                         state.velocities.data());
    }
    compute_all_neighbhorhoods(config, state, temp);

    temp.densities.resize(count);
    predict_densities(count,
                      config.mass_per_particle,
                      config.params.support,
                      temp.densities.data(),
                      temp.neighborhoods.data(),
                      state.positions.data());

    temp.pressures.resize(count, 0.0f);
    init_pressure(count, temp.pressures.data());

    update_pressures(count,
                     config.params.target_density,
                     config.pressure_correction_denom,
                     temp.pressures.data(),
                     temp.densities.data());

    temp.pressure_forces.resize(count, V2f{0.0f, 0.0f});
    compute_pressure_forces(count,
                            config.mass_per_particle,
                            config.params.support,
                            config.params.viscosity,
                            temp.pressure_forces.data(),
                            temp.neighborhoods.data(),
                            state.positions.data(),
                            state.velocities.data(),
                            temp.pressures.data(),
                            temp.densities.data());

    std::vector<V2f> pressure_forces2;
    pressure_forces2.resize(count);
    for (int i = 0; i < REPEATABILITY_TEST_COUNT; ++i) {
        fill_array(count, {0.0f, 0.0f}, pressure_forces2.data());
        compute_pressure_forces(count,
                                config.mass_per_particle,
                                config.params.support,
                                config.params.viscosity,
                                pressure_forces2.data(),
                                temp.neighborhoods.data(),
                                state.positions.data(),
                                state.velocities.data(),
                                temp.pressures.data(),
                                temp.densities.data());

        ASSERT_EQ(temp.pressure_forces, pressure_forces2);
    }
}

TEST_F(Repeatability_test, Sub_step_repeatability) {
    auto const count = state.positions.size();
    Temp_data temp;
    State next_state{state};
    sub_step(config, next_state, temp);

    Temp_data temp2;
    State state2;
    for (int i = 0; i < REPEATABILITY_TEST_COUNT; ++i) {
        state2 = state;
        sub_step(config, state2, temp2);

        if (next_state.positions != state2.positions) {
            fmt::print("Mismatch at iteration: {}\n", i);

            for (size_t j = 0; j < count; ++j) {
                if (next_state.positions[j] != state2.positions[j]) {
                    fmt::print("Mismatch at position index: {}, {}, {}\n",
                               j,
                               next_state.positions[j],
                               state2.positions[j]);
                }
            }
        }

        ASSERT_EQ(next_state.positions, state2.positions);
        ASSERT_EQ(next_state.velocities, state2.velocities);
    }
}
#endif

}  // namespace emerald::sph2d_box