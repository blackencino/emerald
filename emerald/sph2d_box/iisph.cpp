#include <emerald/sph2d_box/iisph.h>

#include <emerald/sph2d_box/iisph_ops.h>
#include <emerald/sph_common/cfl.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/density.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/pressure.h>
#include <emerald/sph_common/volume.h>
#include <emerald/util/assert.h>
#include <emerald/util/format.h>
#include <emerald/util/safe_divide.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

void iisph_sub_step(float const dt,
                    Simulation_config const& config,
                    State& state,
                    Solid_state const& solid_state,
                    Temp_data& temp) {
    auto const particle_count = state.positions.size();

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);
    compute_all_external_forces(config, state, temp);

    compute_densities(particle_count,
                      config.params.support,
                      config.params.target_density,
                      temp.densities.data(),
                      temp.fluid_volumes.data(),
                      temp.neighborhood.pointers(),
                      solid_state.volumes.data(),
                      temp.solid_neighborhood.pointers());

    // HACK
    integrate_velocities_in_place(particle_count,
                                  dt,
                                  config.params.target_density,
                                  state.velocities.data(),
                                  temp.fluid_volumes.data(),
                                  temp.external_forces.data());

    iisph_compute_diis(particle_count,
                       dt,
                       config.params.target_density,
                       temp.diis.data(),
                       temp.densities.data(),
                       temp.fluid_volumes.data(),
                       temp.neighborhood.pointers(),
                       solid_state.volumes.data(),
                       temp.solid_neighborhood.pointers());

    iisph_compute_aiis_and_density_stars(particle_count,
                                         dt,
                                         config.params.target_density,
                                         temp.aiis.data(),
                                         temp.density_stars.data(),
                                         temp.diis.data(),
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
        iisph_compute_sum_dij_pjs(particle_count,
                                  dt,
                                  config.params.target_density,
                                  temp.sum_dij_pjs.data(),
                                  temp.fluid_volumes.data(),
                                  temp.densities.data(),
                                  temp.pressures.data(),
                                  temp.neighborhood.pointers());

        float* betas = temp.new_pressures.data();
        iisph_compute_betas(particle_count,
                            dt,
                            config.params.target_density,
                            betas,
                            temp.densities.data(),
                            temp.pressures.data(),
                            temp.diis.data(),
                            temp.sum_dij_pjs.data(),
                            temp.fluid_volumes.data(),
                            temp.neighborhood.pointers(),
                            solid_state.volumes.data(),
                            temp.solid_neighborhood.pointers());

        auto const [error_average, error_max] =
          iisph_compute_new_pressures(particle_count,
                                      config.params.iisph.omega,
                                      config.params.target_density,
                                      betas,
                                      temp.pressures.data(),
                                      temp.aiis.data(),
                                      temp.density_stars.data());

        std::swap(temp.pressures, temp.new_pressures);

        fmt::print("\t Pressure iter: {}, error avg: {}, max: {}\n",
                   iter,
                   error_average,
                   error_max);

        if (iter > 1 &&
            error_average <= config.params.iisph.error_average_threshold &&
            error_max <= config.params.iisph.error_max_threshold) {
            fmt::print(
              "IISPH pressure solve early exit. err avg: {}, max: {}\n",
              error_average,
              error_max);
            break;
        }
    }
    fmt::print("IISPH pressure iters: {}\n", iter);

    apply_pressures_in_place(particle_count,
                             dt,
                             config.params.target_density,
                             state.velocities.data(),
                             temp.densities.data(),
                             temp.pressures.data(),
                             temp.fluid_volumes.data(),
                             temp.neighborhood.pointers(),
                             solid_state.volumes.data(),
                             temp.solid_neighborhood.pointers());
    integrate_positions_in_place(
      particle_count, dt, state.positions.data(), state.velocities.data());
}

void iisph_resize_temp_arrays(size_t const particle_count, Temp_data& temp) {
    temp.external_forces.resize(particle_count);
    temp.neighborhood.resize(particle_count);
    temp.solid_neighborhood.resize(particle_count);
    temp.densities.resize(particle_count);
    temp.density_stars.resize(particle_count);
    temp.fluid_volumes.resize(particle_count);
    temp.diis.resize(particle_count);
    temp.aiis.resize(particle_count);
    temp.pressures.resize(particle_count);
    temp.new_pressures.resize(particle_count);
    temp.sum_dij_pjs.resize(particle_count);
}

State iisph_simulation_step(Simulation_config const& config,
                            State&& state,
                            Solid_state const& solid_state,
                            Temp_data& temp) {
    flicks const min_sub_time_step = config.params.time_per_step / 30;
    flicks const max_sub_time_step = config.params.time_per_step / 4;

    auto const particle_count = state.positions.size();
    iisph_resize_temp_arrays(particle_count, temp);
    compute_constant_volumes(particle_count,
                             config.params.target_density,
                             config.mass_per_particle,
                             temp.fluid_volumes.data());

    // for (auto const vel : state.velocities) {
    //     fmt::print("Velocity: {}\n", vel);
    // }

    int sub_steps = 0;
    flicks remaining_time_step = config.params.time_per_step;
    while (remaining_time_step.count() > 0) {
        auto const cfl_step = cfl_maximum_time_step(
          particle_count, config.params.support, 0.4f, state.velocities.data());
        // CJH HACK
        // if (cfl_step < min_time_step) {
        //     fmt::print(stderr,
        //                "WARNING: CFL max time step is too small: {}\n",
        //                to_seconds(cfl_step));
        // }
        // auto sub_time_step =
        //   std::clamp(cfl_step, min_time_step, remaining_time_step);
        //
        auto sub_time_step =
          std::clamp(cfl_step, min_sub_time_step, max_sub_time_step);
        sub_time_step = std::min(sub_time_step, remaining_time_step);
        if ((remaining_time_step - sub_time_step) < min_sub_time_step) {
            sub_time_step = remaining_time_step;
        }
        // HACK
        sub_time_step = min_sub_time_step;
        EMLD_ASSERT(
          std::clamp(sub_time_step, min_sub_time_step, max_sub_time_step) ==
            sub_time_step,
          "TIME DISCRETIZATION ERROR");
        fmt::print("Remaining time step: {}, sub time step: {}\n",
                   to_seconds(remaining_time_step),
                   to_seconds(sub_time_step));
        iisph_sub_step(
          to_seconds(sub_time_step), config, state, solid_state, temp);

        remaining_time_step -= sub_time_step;
        ++sub_steps;

        // HACK
        // break;
    }

    state.colors.resize(particle_count);
    compute_colors(particle_count,
                   config.params.target_density,
                   state.colors.data(),
                   temp.densities.data());

    fmt::print("IISPH frame complete, sub_steps: {}\n", sub_steps);

    return std::move(state);
}

}  // namespace emerald::sph2d_box
