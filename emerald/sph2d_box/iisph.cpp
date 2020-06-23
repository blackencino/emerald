#include <emerald/sph2d_box/iisph.h>

#include <emerald/sph2d_box/adaptive_time_step.h>
#include <emerald/sph2d_box/iisph_ops.h>
#include <emerald/sph2d_box/neighborhoods_from_state.h>
#include <emerald/sph_common/cfl.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/density.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/pressure.h>
#include <emerald/util/assert.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

static void iisph_sub_step(float const global_time_in_seconds,
                           float const dt,
                           Simulation_config const& config,
                           State& state,
                           Solid_state const& solid_state,
                           Temp_data& temp,
                           User_forces_function const& user_forces) {
    auto const particle_count = state.positions.size();

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);
    compute_all_external_forces(global_time_in_seconds,
                                dt,
                                config,
                                state,
                                solid_state,
                                temp,
                                user_forces);

    compute_densities(particle_count,
                      config.params.support,
                      config.params.target_density,
                      temp.densities.data(),
                      temp.fluid_volumes.data(),
                      temp.neighborhood.pointers(),
                      solid_state.volumes.data(),
                      temp.solid_neighborhood.pointers());

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

        // fmt::print("\t Pressure iter: {}, error avg: {}, max: {}\n",
        //            iter,
        //            error_average,
        //            error_max);

        if (iter > 1 &&
            error_average <= config.params.iisph.error_average_threshold &&
            error_max <= config.params.iisph.error_max_threshold) {
            // fmt::print(
            //   "IISPH pressure solve early exit. err avg: {}, max: {}\n",
            //   error_average,
            //   error_max);
            break;
        }
    }
    // fmt::print("IISPH pressure iters: {}\n", iter);

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

static void iisph_resize_temp_arrays(size_t const particle_count,
                                     Temp_data& temp) {
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

State iisph_simulation_step(flicks const global_time,
                            Simulation_config const& config,
                            State&& state,
                            Solid_state const& solid_state,
                            Temp_data& temp,
                            User_forces_function const& user_forces,
                            User_colors_function const& user_colors) {
    return std_adaptive_time_step("IISPH",
                                  iisph_resize_temp_arrays,
                                  iisph_sub_step,
                                  60,
                                  6,
                                  0.2f,
                                  global_time,
                                  config,
                                  std::move(state),
                                  solid_state,
                                  temp,
                                  user_forces,
                                  user_colors);
}

}  // namespace emerald::sph2d_box
