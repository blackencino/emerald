#include <emerald/sph2d_box/dfsph_p.h>

#include <emerald/sph2d_box/dfsph.h>
#include <emerald/sph2d_box/dfsph_ops.h>
#include <emerald/sph2d_box/dfsph_p_ops.h>
#include <emerald/sph2d_box/iisph.h>
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

void dfsph_p_correct_density_error(float const dt,
                                   Simulation_config const& config,
                                   State& state,
                                   Solid_state const& solid_state,
                                   Temp_data& temp) {
    auto const particle_count = state.positions.size();

    // Warm start
    dfsph_p_integrate_pseudo_pressures(particle_count,
                                       dt,
                                       config.params.target_density,

                                       state.velocities.data(),
                                       temp.pressure_accelerations.data(),
                                       temp.density_kappas.data(),
                                       temp.densities.data(),
                                       temp.fluid_volumes.data(),
                                       temp.neighborhood.pointers(),
                                       solid_state.volumes.data(),
                                       temp.solid_neighborhood.pointers());

    for (int iter = 0; iter < config.params.dfsph.max_correction_iterations;
         ++iter) {
        dfsph_p_compute_density_stars(particle_count,
                                      dt,
                                      config.params.target_density,
                                      temp.density_stars.data(),
                                      temp.densities.data(),
                                      temp.fluid_volumes.data(),
                                      state.velocities.data(),
                                      temp.neighborhood.pointers(),
                                      solid_state.volumes.data(),
                                      solid_state.velocities.data(),
                                      temp.solid_neighborhood.pointers());

        dfsph_p_compute_density_pseudo_pressures(particle_count,
                                                 dt,
                                                 config.params.target_density,
                                                 temp.density_kappas.data(),
                                                 temp.density_stars.data(),
                                                 temp.alphas.data());

        dfsph_p_integrate_pseudo_pressures(particle_count,
                                           dt,
                                           config.params.target_density,

                                           state.velocities.data(),
                                           temp.pressure_accelerations.data(),
                                           temp.density_kappas.data(),
                                           temp.densities.data(),
                                           temp.fluid_volumes.data(),
                                           temp.neighborhood.pointers(),
                                           solid_state.volumes.data(),
                                           temp.solid_neighborhood.pointers());

        if (iter > 1) {
            auto const [error_avg, error_max] =
              compute_transformed_scalar_average_and_max(
                particle_count,
                1000.0f,
                temp.density_stars.data(),
                [target_density =
                   config.params.target_density](float const density_star) {
                    auto const error = density_star - target_density;
                    return error <= 0.0f ? 0.0f : (error / target_density);
                });
            // fmt::print("Correct density error. iter: {}, avg: {}, max: {}\n",
            //            iter,
            //            error_avg,
            //            error_max);
            if (error_avg <=
                  config.params.dfsph.density_error_average_threshold &&
                error_max <= config.params.dfsph.density_error_max_threshold) {
                break;
            }
        }
    }
}

void dfsph_p_correct_divergence_error(float const dt,
                                      Simulation_config const& config,
                                      State& state,
                                      Solid_state const& solid_state,
                                      Temp_data& temp) {
    auto const particle_count = state.positions.size();

    // Warm start
    dfsph_p_integrate_pseudo_pressures(particle_count,
                                       dt,
                                       config.params.target_density,

                                       state.velocities.data(),
                                       temp.pressure_accelerations.data(),
                                       temp.divergence_kappas.data(),
                                       temp.densities.data(),
                                       temp.fluid_volumes.data(),
                                       temp.neighborhood.pointers(),
                                       solid_state.volumes.data(),
                                       temp.solid_neighborhood.pointers());

    for (int iter = 0; iter < config.params.dfsph.max_correction_iterations;
         ++iter) {
        dfsph_p_compute_divergences(particle_count,
                                    config.params.target_density,
                                    temp.density_stars.data(),
                                    temp.fluid_volumes.data(),
                                    state.velocities.data(),
                                    temp.neighborhood.pointers(),
                                    solid_state.volumes.data(),
                                    solid_state.velocities.data(),
                                    temp.solid_neighborhood.pointers());

        dfsph_p_compute_divergence_pseudo_pressures(
          particle_count,
          dt,
          config.params.target_density,
          temp.divergence_kappas.data(),
          temp.density_stars.data(),
          temp.alphas.data());

        dfsph_p_integrate_pseudo_pressures(particle_count,
                                           dt,
                                           config.params.target_density,
                                           state.velocities.data(),
                                           temp.pressure_accelerations.data(),
                                           temp.divergence_kappas.data(),
                                           temp.densities.data(),
                                           temp.fluid_volumes.data(),
                                           temp.neighborhood.pointers(),
                                           solid_state.volumes.data(),
                                           temp.solid_neighborhood.pointers());

        if (iter > 0) {
            auto const [error_avg, error_max] =
              compute_transformed_scalar_average_and_max(
                particle_count,
                1000.0f,
                temp.density_stars.data(),
                [dt, target_density = config.params.target_density](
                  float const divergence) {
                    return divergence <= 0.0f
                             ? 0.0f
                             : (dt * divergence / target_density);
                });

            // fmt::print("Correct divergence error. iter: {}, avg: {}, max:
            // {}\n",
            //            iter,
            //            error_avg,
            //            error_max);
            if (error_avg <=
                  config.params.dfsph.divergence_error_average_threshold &&
                error_max <=
                  config.params.dfsph.divergence_error_max_threshold) {
                break;
            }
        }
    }
}

void dfsph_p_resize_and_init_temp_arrays(size_t const particle_count,
                                         Temp_data& temp) {
    temp.external_forces.resize(particle_count);
    temp.alphas.resize(particle_count);
    temp.neighborhood.resize(particle_count);
    temp.solid_neighborhood.resize(particle_count);
    temp.densities.resize(particle_count);
    temp.divergence_kappas.resize(particle_count);
    temp.density_kappas.resize(particle_count);
    temp.density_stars.resize(particle_count);
    temp.pressure_accelerations.resize(particle_count);
    temp.fluid_volumes.resize(particle_count);

    fill_array(particle_count, 0.0f, temp.divergence_kappas.data());
    fill_array(particle_count, 0.0f, temp.density_kappas.data());
}

void dfsph_p_init(Simulation_config const& config,
                  State const& state,
                  const Solid_state& solid_state,
                  Temp_data& temp) {
    auto const particle_count = state.positions.size();
    dfsph_p_resize_and_init_temp_arrays(particle_count, temp);
    iisph_compute_fluid_volumes(particle_count,
                                config.params.target_density,
                                config.mass_per_particle,
                                temp.fluid_volumes.data());
    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);

    iisph_compute_densities(particle_count,
                            config.params.support,
                            config.params.target_density,
                            temp.densities.data(),
                            temp.fluid_volumes.data(),
                            temp.neighborhood.pointers(),
                            solid_state.volumes.data(),
                            temp.solid_neighborhood.pointers());

    dfsph_p_compute_shared_coeffs(particle_count,
                                  config.params.target_density,
                                  temp.alphas.data(),
                                  temp.densities.data(),
                                  temp.fluid_volumes.data(),
                                  temp.neighborhood.pointers(),
                                  solid_state.volumes.data(),
                                  temp.solid_neighborhood.pointers());
}

void dfsph_p_sub_step(float const dt,
                      Simulation_config const& config,
                      State& state,
                      Solid_state const& solid_state,
                      Temp_data& temp) {
    auto const particle_count = state.positions.size();

    // External forces passed in
    iisph_integrate_velocities_in_place(particle_count,
                                        dt,
                                        config.params.target_density,
                                        state.velocities.data(),
                                        temp.fluid_volumes.data(),
                                        temp.external_forces.data());

    dfsph_p_correct_density_error(dt, config, state, solid_state, temp);

    iisph_integrate_positions_in_place(
      particle_count, dt, state.positions.data(), state.velocities.data());
    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);

    iisph_compute_densities(particle_count,
                            config.params.support,
                            config.params.target_density,
                            temp.densities.data(),
                            temp.fluid_volumes.data(),
                            temp.neighborhood.pointers(),
                            solid_state.volumes.data(),
                            temp.solid_neighborhood.pointers());

    dfsph_p_compute_shared_coeffs(particle_count,
                                  config.params.target_density,
                                  temp.alphas.data(),
                                  temp.densities.data(),
                                  temp.fluid_volumes.data(),
                                  temp.neighborhood.pointers(),
                                  solid_state.volumes.data(),
                                  temp.solid_neighborhood.pointers());

    dfsph_p_correct_divergence_error(dt, config, state, solid_state, temp);

    // Velocity is updated in place.
}

State dfsph_p_simulation_step(Simulation_config const& config,
                              State&& state,
                              const Solid_state& solid_state,
                              Temp_data& temp) {
    auto const start = std::chrono::high_resolution_clock::now();

    auto const particle_count = state.positions.size();
    dfsph_p_resize_and_init_temp_arrays(particle_count, temp);
    iisph_compute_fluid_volumes(particle_count,
                                config.params.target_density,
                                config.mass_per_particle,
                                temp.fluid_volumes.data());

    flicks const min_sub_time_step = config.params.time_per_step / 60;
    flicks const max_sub_time_step = config.params.time_per_step / 4;

    int remaining_sub_steps = 60;
    flicks const time_per_sub_step =
      config.params.time_per_step / remaining_sub_steps;
    int sub_steps = 0;
    while (remaining_sub_steps > 0) {
        auto const cfl_step = iisph_cfl_maximum_time_step(
          particle_count, config.params.support, state.velocities.data());

        auto num_sub_steps = static_cast<int>(
          std::ceil(to_seconds(cfl_step) / to_seconds(time_per_sub_step)));
        num_sub_steps = std::clamp(num_sub_steps, 1, 15);
        num_sub_steps = std::min(num_sub_steps, remaining_sub_steps);

        auto const sub_time_step = num_sub_steps * time_per_sub_step;

        EMLD_ASSERT(
          std::clamp(sub_time_step, min_sub_time_step, max_sub_time_step) ==
            sub_time_step,
          "TIME DISCRETIZATION ERROR");

        compute_all_external_forces(config, state, temp);

        dfsph_p_sub_step(
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

    fmt::print("DFSPH P frame complete, sub_steps: {}, ms: {}\n",
               sub_steps,
               std::chrono::duration<double>{end - start}.count() * 1000.0);

    return std::move(state);
}

}  // namespace emerald::sph2d_box
