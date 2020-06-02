#include <emerald/sph2d_box/iisph_pseudo_ap.h>

#include <emerald/sph2d_box/iisph_ap.h>
#include <emerald/sph2d_box/iisph_ap_ops.h>
#include <emerald/sph2d_box/iisph_ops.h>
#include <emerald/sph2d_box/iisph_pseudo_ap_ops.h>
#include <emerald/sph2d_box/sim_ops.h>
#include <emerald/sph_common/common.h>
#include <emerald/util/assert.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>
#include <emerald/util/safe_divide.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

void iisph_pseudo_ap_sub_step(float const dt,
                              Simulation_config const& config,
                              State& state,
                              Solid_state const& solid_state,
                              Temp_data& temp) {
    auto const particle_count = state.positions.size();

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);
    compute_all_external_forces(config, state, temp);

    iisph_compute_densities(particle_count,
                            config.params.support,
                            config.params.target_density,
                            temp.densities.data(),
                            temp.fluid_volumes.data(),
                            temp.neighborhood.pointers(),
                            solid_state.volumes.data(),
                            temp.solid_neighborhood.pointers());

    iisph_integrate_velocities_in_place(particle_count,
                                        dt,
                                        config.params.target_density,
                                        state.velocities.data(),
                                        temp.fluid_volumes.data(),
                                        temp.external_forces.data());

    iisph_pseudo_ap_density_stars_and_pseudo_diagonals(
      particle_count,
      dt,
      config.params.target_density,
      temp.density_stars.data(),
      temp.aiis.data(),
      temp.densities.data(),
      temp.fluid_volumes.data(),
      state.velocities.data(),
      temp.neighborhood.pointers(),
      solid_state.volumes.data(),
      solid_state.velocities.data(),
      temp.solid_neighborhood.pointers());

    fill_array(particle_count, 0.0f, temp.pressures.data());

    int iter = 0;
    for (; iter < config.params.iisph.max_pressure_iterations; ++iter) {
        iisph_pseudo_ap_compute_pseudo_pressure_displacements(
          particle_count,
          config.params.target_density,
          temp.pressure_accelerations.data(),
          temp.pressures.data(),
          temp.densities.data(),
          temp.fluid_volumes.data(),
          temp.neighborhood.pointers(),
          solid_state.volumes.data(),
          temp.solid_neighborhood.pointers());

        auto const [error_average, error_max] =
          iisph_pseudo_ap_iterate_pseudo_pressures_in_place(
            particle_count,
            config.params.target_density,
            config.params.iisph.omega,
            temp.pressures.data(),
            temp.density_stars.data(),
            temp.pressure_accelerations.data(),
            temp.aiis.data(),
            temp.fluid_volumes.data(),
            temp.neighborhood.pointers(),
            solid_state.volumes.data(),
            temp.solid_neighborhood.pointers());

        // fmt::print("\t Pseudo Pressure iter: {}, error avg: {}, max: {}\n",
        //            iter,
        //            error_average,
        //            error_max);

        if (iter > 1 &&
            error_average <= config.params.iisph.error_average_threshold &&
            error_max <= config.params.iisph.error_max_threshold) {
            // fmt::print(
            //   "IISPH PSEUDO AP pressure solve early exit. err avg: {}, max: "
            //   "{}\n",
            //   error_average,
            //   error_max);
            break;
        }
    }
    // fmt::print("IISPH PSEUDO AP pressure iters: {}\n", iter);

    iisph_pseudo_ap_compute_pseudo_pressure_displacements(
      particle_count,
      config.params.target_density,
      temp.pressure_accelerations.data(),
      temp.pressures.data(),
      temp.densities.data(),
      temp.fluid_volumes.data(),
      temp.neighborhood.pointers(),
      solid_state.volumes.data(),
      temp.solid_neighborhood.pointers());

    iisph_pseudo_ap_integrate_velocities_and_positions_in_place(
      particle_count,
      dt,
      state.velocities.data(),
      state.positions.data(),
      temp.pressure_accelerations.data());
}

State iisph_pseudo_ap_simulation_step(Simulation_config const& config,
                                      State&& state,
                                      const Solid_state& solid_state,
                                      Temp_data& temp) {
    auto const start = std::chrono::high_resolution_clock::now();
    flicks const min_time_step = config.params.time_per_step / 1000;

    auto const particle_count = state.positions.size();
    iisph_ap_resize_temp_arrays(particle_count, temp);
    iisph_compute_fluid_volumes(particle_count,
                                config.params.target_density,
                                config.mass_per_particle,
                                temp.fluid_volumes.data());

    // for (auto const vel : state.velocities) {
    //     fmt::print("Velocity: {}\n", vel);
    // }

    flicks const min_sub_time_step = config.params.time_per_step / 30;
    flicks const max_sub_time_step = config.params.time_per_step / 5;

    int remaining_sub_steps = 30;
    flicks const time_per_sub_step =
      config.params.time_per_step / remaining_sub_steps;
    int sub_steps = 0;
    while (remaining_sub_steps > 0) {
        auto const cfl_step =
          iisph_cfl_maximum_time_step(
            particle_count, config.params.support, state.velocities.data()) /
          2;

        auto num_sub_steps = static_cast<int>(
          std::ceil(to_seconds(cfl_step) / to_seconds(time_per_sub_step)));
        num_sub_steps = std::clamp(num_sub_steps, 1, 6);
        num_sub_steps = std::min(num_sub_steps, remaining_sub_steps);

        auto const sub_time_step = num_sub_steps * time_per_sub_step;

        EMLD_ASSERT(
          std::clamp(sub_time_step, min_sub_time_step, max_sub_time_step) ==
            sub_time_step,
          "TIME DISCRETIZATION ERROR");

        iisph_pseudo_ap_sub_step(
          to_seconds(sub_time_step), config, state, solid_state, temp);

        remaining_sub_steps -= num_sub_steps;
        ++sub_steps;
    }

    state.colors.resize(particle_count);
    compute_colors(particle_count,
                   config.params.target_density,
                   state.colors.data(),
                   temp.densities.data());

    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("IISPH PSEUDO AP frame complete, sub_steps: {}, ms: {}\n",
               sub_steps,
               std::chrono::duration<double>{end - start}.count() * 1000.0);

    return std::move(state);
}

}  // namespace emerald::sph2d_box