#include <emerald/sph2d_box/iisph_ap.h>

#include <emerald/sph2d_box/iisph_ap_ops.h>
#include <emerald/sph2d_box/iisph_ops.h>
#include <emerald/sph2d_box/sim_ops.h>
#include <emerald/sph_common/common.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>
#include <emerald/util/safe_divide.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

void iisph_ap_sub_step(float const dt,
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

    iisph_ap_density_stars_and_diagonals(particle_count,
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
        iisph_ap_compute_pressure_accelerations(
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
          iisph_ap_iterate_pressures_in_place(
            particle_count,
            dt,
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

        fmt::print("\t Pressure iter: {}, error avg: {}, max: {}\n",
                   iter,
                   error_average,
                   error_max);

        if (iter > 1 &&
            error_average <= config.params.iisph.error_average_threshold &&
            error_max <= config.params.iisph.error_max_threshold) {
            fmt::print(
              "IISPH AP pressure solve early exit. err avg: {}, max: {}\n",
              error_average,
              error_max);
            break;
        }
    }
    fmt::print("IISPH AP pressure iters: {}\n", iter);

    iisph_ap_compute_pressure_accelerations(particle_count,
                                            config.params.target_density,
                                            temp.pressure_accelerations.data(),
                                            temp.pressures.data(),
                                            temp.densities.data(),
                                            temp.fluid_volumes.data(),
                                            temp.neighborhood.pointers(),
                                            solid_state.volumes.data(),
                                            temp.solid_neighborhood.pointers());

    iisph_ap_integrate_velocities_and_positions_in_place(
      particle_count,
      dt,
      state.velocities.data(),
      state.positions.data(),
      temp.pressure_accelerations.data());
}

void iisph_ap_resize_temp_arrays(size_t const particle_count, Temp_data& temp) {
    temp.external_forces.resize(particle_count);
    temp.neighborhood.resize(particle_count);
    temp.solid_neighborhood.resize(particle_count);
    temp.densities.resize(particle_count);
    temp.density_stars.resize(particle_count);
    temp.fluid_volumes.resize(particle_count);
    temp.aiis.resize(particle_count);
    temp.pressures.resize(particle_count);
    temp.pressure_accelerations.resize(particle_count);
}

State iisph_ap_simulation_step(Simulation_config const& config,
                               State&& state,
                               const Solid_state& solid_state,
                               Temp_data& temp) {
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

    int sub_steps = 0;
    flicks remaining_time_step = config.params.time_per_step;
    while (remaining_time_step.count() > 0) {
        auto const cfl_step = iisph_cfl_maximum_time_step(
          particle_count, config.params.support, state.velocities.data());

        auto const cfl_ratio =
          to_seconds(config.params.time_per_step) / to_seconds(cfl_step);
        fmt::print("CFL RATIO: {}\n", cfl_ratio);
        // CJH HACK
        // if (cfl_step < min_time_step) {
        //     fmt::print(stderr,
        //                "WARNING: CFL max time step is too small: {}\n",
        //                to_seconds(cfl_step));
        // }
        // auto sub_time_step =
        //   std::clamp(cfl_step, min_time_step, remaining_time_step);
        // if ((remaining_time_step - sub_time_step) < min_time_step) {
        //     sub_time_step = flicks{(remaining_time_step.count() + 1) / 2};
        // }
        // auto const sub_time_step = std::min(cfl_step, remaining_time_step);
        // auto const sub_time_step = remaining_time_step;
        auto const sub_time_step = config.params.time_per_step / 10;
        fmt::print("Remaining time step: {}, sub time step: {}\n",
                   to_seconds(remaining_time_step),
                   to_seconds(sub_time_step));
        iisph_ap_sub_step(
          to_seconds(sub_time_step), config, state, solid_state, temp);

        remaining_time_step -= sub_time_step;
        ++sub_steps;
    }

    state.colors.resize(particle_count);
    compute_colors(particle_count,
                   config.params.target_density,
                   state.colors.data(),
                   temp.densities.data());

    fmt::print("IISPH AP frame complete, sub_steps: {}\n", sub_steps);

    return std::move(state);
}

}  // namespace emerald::sph2d_box