#include <emerald/sph2d_box/pcisph.h>

#include <emerald/sph2d_box/neighborhoods_from_state.h>
#include <emerald/sph2d_box/pcisph_ops.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/kernels.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

namespace emerald::sph2d_box {

void pcisph_sub_step(float const global_time_in_seconds,
                     float const dt,
                     Simulation_config const& config,
                     State& state,
                     Solid_state const& solid_state,
                     Temp_data& temp,
                     User_forces_function const& user_forces) {
    auto const count = state.positions.size();

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_external_forces(global_time_in_seconds,
                                dt,
                                config,
                                state,
                                solid_state,
                                temp,
                                user_forces);

    // Integrate velocity only from external force
    temp.velocity_external_forces.resize(count);
    copy_array(
      count, temp.velocity_external_forces.data(), state.velocities.data());
    accumulate(count,
               dt / config.mass_per_particle,
               temp.velocity_external_forces.data(),
               temp.external_forces.data());

    // Init for solving pressures
    temp.pressures.resize(count);
    fill_array(count, 0.0f, temp.pressures.data());

    temp.pressure_forces.resize(count);
    temp.densities.resize(count);
    temp.position_stars.resize(count);
    temp.velocity_stars.resize(count);

    for (int pci_sub_step = 0; pci_sub_step < 6; ++pci_sub_step) {
        copy_array(count,
                   temp.velocity_stars.data(),
                   temp.velocity_external_forces.data());

        if (pci_sub_step > 0) {
            // integrate velocity
            accumulate(count,
                       dt / config.mass_per_particle,
                       temp.velocity_stars.data(),
                       temp.pressure_forces.data());
        }

        // integrate position
        copy_array(count, temp.position_stars.data(), state.positions.data());
        accumulate(
          count, dt, temp.position_stars.data(), temp.velocity_stars.data());

        // Recompute neighborhood values with kernels.
        recompute_neighborhood_non_index_values(config, solid_state, temp);

        fill_array(count, {0.0f, 0.0f}, temp.pressure_forces.data());

        pcisph_compute_densities(count,
                                 config.mass_per_particle,
                                 config.params.support,
                                 temp.densities.data(),
                                 temp.neighborhood.counts.data(),
                                 temp.neighborhood.kernels.data());

        pcisph_accumulate_density_from_solids(
          count,
          config.params.target_density,
          temp.densities.data(),
          solid_state.volumes.data(),
          temp.solid_neighborhood.counts.data(),
          temp.solid_neighborhood.indices.data(),
          temp.solid_neighborhood.kernels.data());

        pcisph_update_pressures(count,
                                config.params.target_density,
                                config.pressure_correction_denom,
                                temp.pressures.data(),
                                temp.densities.data());

        pcisph_accumulate_pressure_forces(
          count,
          config.mass_per_particle,
          config.params.support,
          config.params.viscosity,
          temp.pressure_forces.data(),
          temp.neighborhood.counts.data(),
          temp.neighborhood.indices.data(),
          temp.neighborhood.distances.data(),
          temp.neighborhood.vectors_to.data(),
          temp.neighborhood.kernel_gradients.data(),
          temp.velocity_stars.data(),
          temp.pressures.data(),
          temp.densities.data());

        pcisph_accumulate_pressure_forces_from_solids(
          count,
          config.mass_per_particle,
          config.params.target_density,
          temp.pressure_forces.data(),
          solid_state.volumes.data(),
          temp.solid_neighborhood.counts.data(),
          temp.solid_neighborhood.indices.data(),
          temp.solid_neighborhood.kernel_gradients.data(),
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

    // Integrate velocity
    copy_array(
      count, state.velocities.data(), temp.velocity_external_forces.data());
    accumulate(count,
               dt / config.mass_per_particle,
               state.velocities.data(),
               temp.pressure_forces.data());

    // Integrate position
    accumulate(count, dt, state.positions.data(), state.velocities.data());
}

State pcisph_simulation_step(flicks const global_time,
                             Simulation_config const& config,
                             State&& state,
                             Solid_state const& solid_state,
                             Temp_data& temp,
                             User_forces_function const& user_forces,
                             User_colors_function const& user_colors) {
    auto const start = std::chrono::high_resolution_clock::now();

    auto const flicks_per_sub_step =
      config.params.time_per_step / config.params.sub_steps;
    auto time = global_time;
    for (int sub_step_iter = 0; sub_step_iter < config.params.sub_steps;
         ++sub_step_iter) {
        pcisph_sub_step(to_seconds(time),
                        config.seconds_per_sub_step,
                        config,
                        state,
                        solid_state,
                        temp,
                        user_forces);
        time += flicks_per_sub_step;
    }

    compute_colors(
      to_seconds(time), config, state, solid_state, temp, user_colors);

    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("PCISPH frame complete, sub_steps: {}, ms: {}\n",
               config.params.sub_steps,
               std::chrono::duration<double>{end - start}.count() * 1000.0);

    return std::move(state);
}

}  // namespace emerald::sph2d_box
