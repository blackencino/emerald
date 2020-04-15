#include <emerald/sph2d_box/simulation.h>

#include <emerald/sph2d_box/foundation.h>
#include <emerald/sph2d_box/kernels.h>
#include <emerald/util/functions.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace emerald::sph2d_box {

Simulation_config::Simulation_config(Parameters const& in_params)
  : params(in_params) {
    // The grid cells are H*2 on a side, where "H" is support.
    float const H = params.support;
    float const Hhalf = H * 0.5f;
    float const R = 0.99f * Hhalf;
    draw_radius = R;

    // Create a representative, fully-packed volume and calculate the
    // muchness0 and the subsequent per-particle mass and volume.
    // Also use this opportunity to calculate the denominator of the
    // density correction function.
    V2f gradw_sum{0.0f, 0.0f};
    float gradw_dot_gradw_sum = 0.0f;
    float muchness0 = 0.0f;
    float const kernel0 = kernels::W(0.0f, H);
    V2f const pos_i{0.0f * R, 0.588457f * R};

    int num_neighbors = 0;

    // -15 to 15 is more than we need, but just to be safe...
    V2f const init_volume_min{-15.0f * R, -15.0f};
    V2f const init_volume_max{15.0f * R, 15.0f};

    float min_length = 10000.0f;
    V2f min_length_point;
    V2f min_length_delta_position;

    // Do the cube
    V2f const start_point = init_volume_min;
    V2f point = start_point;
    bool push_x = false;
    float offset_x = 0.0f;
    float const dx = 2.0f * R;
    float const dy = std::sqrt(3.0f) * R;
    for (; point.y <= init_volume_max.y; point.y += dy) {
        // Do a row.
        offset_x = R;
        point.x = push_x ? start_point.x : start_point.x + R;
        point.x += offset_x;
        for (; point.x <= init_volume_max.x; point.x += dx) {
            auto const delta_position = pos_i - point;
            auto const length = delta_position.length();

            if (length < min_length) {
                min_length = length;
                min_length_point = point;
                min_length_delta_position = delta_position;
            }

            auto const w = kernels::W(length, H);
            if (w > 0.0f) {
                muchness0 += w;
                ++num_neighbors;
            }

            auto const gradw = kernels::GradW(delta_position, H);

            gradw_sum += gradw;
            gradw_dot_gradw_sum += gradw.dot(gradw);
        }
        push_x = !push_x;
    }

    fmt::print(
        "Min R = {}\n"
        "Min R point = ({}, {})\n"
        "Min R delta_position = ({}, {})\n"
        "num neighbors = {}\n",
        min_length,
        min_length_point[0],
        min_length_point[1],
        min_length_delta_position[0],
        min_length_delta_position[1],
        num_neighbors);

    // Density of a particle is equal to mass times muchness.
    // So density, here, is muchness0 * m_massPerParticle.
    // therefore, m_massPerParticle = kDensityOfWater / muchness0;
    mass_per_particle = params.target_density / muchness0;

    fmt::print(
        "Muchness0: {}\n"
        "Mass per particle: {}\n",
        muchness0,
        mass_per_particle);

    seconds_per_sub_step = to_seconds(params.time_per_step / params.sub_steps);

    // Beta is defined as 2.0 * sqr( dt * m / rho0 );
    // However, we just defined m as rho0/muchness.
    // Therefore, beta is just 2.0 * sqr( dt/muchness );
    // float beta = 2.0 * sqr( dt * m_massPerParticle / kDensityOfWater );
    auto const beta = 2.0f * sqr(seconds_per_sub_step / muchness0);

    // I take out the negative here, and later.
    pressure_correction_denom =
        beta * (gradw_sum.dot(gradw_sum) + gradw_dot_gradw_sum);

    fmt::print(
        "Pressure correction denom: {}\n"
        "gradw_sum dot gradw_sum: {}\n"
        "gradw_sum: ({}, {})\n"
        "gradw_dot_gradw_sum: {}\n",
        pressure_correction_denom,
        gradw_sum.dot(gradw_sum),
        gradw_sum[0],
        gradw_sum[1],
        gradw_dot_gradw_sum);
};

State dam_break_initial_state(Parameters const& params) {
    State state;

    // The grid cells are H*2 on a side, where "H" is support.
    float const H = params.support;
    float const Hhalf = H * 0.5f;
    float const R = 0.99f * Hhalf;
    float const L = params.length;

    V2f const init_volume_min{0.0f, 0.0f};
    V2f const init_volume_max{L / 2.0f, 3.25f * L / 4.0f};

    // Do the cube
    V2f start_point = init_volume_min;
    V2f point = start_point;
    bool push_x = false;
    float offset_x = 0.0f;
    float const dx = 2.0f * R;
    float const dy = std::sqrt(3.0f) * R;
    for (; point.y <= init_volume_max.y; point.y += dy) {
        // Do a row.
        offset_x = R;
        point.x = push_x ? start_point.x : start_point.x + R;
        point.x += offset_x;
        for (; point.x <= init_volume_max.x; point.x += dx) {
            state.positions.push_back(point);
            state.velocities.push_back(V2f{0.0f, 0.0f});
            state.colors.push_back(C4uc{128, 128, 255, 255});
        }
        push_x = !push_x;
    }

    return state;
}

static void sub_step(Simulation_config const& config,
                     State& state,
                     Temp_data& temp) {
    //------------------------------------------------------------------------------
    // NEIGHBORHOOD
    auto const count = state.positions.size();
    auto const cell_size = config.params.support * 2.0f;
    temp.grid_coords.resize(count);
    compute_grid_coords(count,
                        cell_size,
                        V2f{0.0f, 0.0f},
                        temp.grid_coords.data(),
                        state.positions.data());

    temp.z_indices.resize(count);
    compute_z_indices(count, temp.z_indices.data(), temp.grid_coords.data());

    temp.index_pairs.resize(count);
    compute_index_pairs(count, temp.index_pairs.data(), temp.z_indices.data());
    sort_index_pairs(count, temp.index_pairs.data());

    temp.block_indices.resize(count);
    compute_block_indices(
        count, temp.block_indices.data(), temp.index_pairs.data());

    size_t const block_count = temp.block_indices.back() + 1;
    temp.blocks.resize(block_count);
    fill_blocks(count, temp.blocks.data(), temp.block_indices.data());

    temp.block_map = create_block_map(block_count,
                                      std::move(temp.block_map),
                                      temp.blocks.data(),
                                      temp.index_pairs.data());

    temp.neighborhoods.resize(count);
    create_neighborhoods(count,
                         cell_size,
                         temp.neighborhoods.data(),
                         state.positions.data(),
                         temp.grid_coords.data(),
                         temp.index_pairs.data(),
                         temp.block_map);

    //------------------------------------------------------------------------------
    // NON PRESSURE FORCE AND PRESSURE INIT
    temp.forces.resize(count);
    compute_external_forces(count,
                            config.mass_per_particle,
                            config.params.gravity,
                            temp.forces.data());

    temp.pressures.resize(count, 0.0f);
    init_pressure(count, temp.pressures.data());

    temp.pressure_forces.resize(count);
    temp.densities.resize(count);
    temp.position_stars.resize(count);
    temp.velocity_stars.resize(count);
    for (int pci_sub_step = 0; pci_sub_step < 6; ++pci_sub_step) {
        predict_velocities(count,
                           config.mass_per_particle,
                           config.seconds_per_sub_step,
                           temp.velocity_stars.data(),
                           state.velocities.data(),
                           temp.pressure_forces.data(),
                           temp.forces.data());

        predict_positions(count,
                          config.seconds_per_sub_step,
                          temp.position_stars.data(),
                          state.positions.data(),
                          temp.velocity_stars.data());

        enforce_solid_boundaries(count,
                                 config.params.support,
                                 config.params.length,
                                 temp.position_stars.data(),
                                 temp.velocity_stars.data());

        predict_densities(count,
                          config.mass_per_particle,
                          config.params.support,
                          temp.densities.data(),
                          temp.neighborhoods.data(),
                          temp.position_stars.data());

        update_pressures(count,
                         config.params.target_density,
                         config.pressure_correction_denom,
                         temp.pressures.data(),
                         temp.densities.data());

        compute_pressure_forces(count,
                                config.mass_per_particle,
                                config.params.support,
                                config.params.viscosity,
                                temp.pressure_forces.data(),
                                temp.neighborhoods.data(),
                                temp.position_stars.data(),
                                temp.velocity_stars.data(),
                                temp.pressures.data(),
                                temp.densities.data());

        if (pci_sub_step > 1) {
            auto const max_error = max_density_error(
                count, config.params.target_density, temp.densities.data());
            if ((max_error / config.params.target_density) <= 0.01f) { break; }
        }
    }

    //------------------------------------------------------------------------------
    // FINAL UPDATE
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

State simulation_step(Simulation_config const& config,
                      State&& state,
                      Temp_data& temp_data) {
    for (int sub_step_iter = 0; sub_step_iter < config.params.sub_steps;
         ++sub_step_iter) {
        sub_step(config, state, temp_data);
    }

    auto const count = state.positions.size();
    state.colors.resize(count);
    compute_colors(count,
                   config.params.target_density,
                   state.colors.data(),
                   temp_data.densities.data());

    return std::move(state);
}

}  // namespace emerald::sph2d_box
