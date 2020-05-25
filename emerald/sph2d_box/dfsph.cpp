#include <emerald/sph2d_box/dfsph.h>

#include <emerald/sph2d_box/dfsph_ops.h>
#include <emerald/sph_common/common.h>
#include <emerald/util/safe_divide.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

flicks dfsph_cfl_maximum_time_step(size_t const particle_count,
                                   float const support,
                                   V2f const* const velocities) {
    double const numer = 0.4 * support;
    double const denom = std::sqrt(static_cast<double>(
      max_vector_squared_magnitude(particle_count, velocities)));

    return to_flicks(safe_divide(numer, denom).value_or(0.0f));
}

void dfsph_integrate_velocity_from_non_pressure_forces(
  float const dt,
  Simulation_config const& config,
  State& state,
  Solid_state const& solid_state,
  Temp_data& temp) {
    auto const count = state.positions.size();
    accumulate(count,
               dt / config.mass_per_particle,
               state.velocities.data(),
               temp.external_forces.data());
}

void dfsph_integrate_positions(float const dt,
                               Simulation_config const& config,
                               State& state,
                               Solid_state const& solid_state,
                               Temp_data& temp) {
    auto const count = state.positions.size();
    accumulate(count, dt, state.positions.data(), state.velocities.data());
}

void dfsph_compute_alphas(Simulation_config const& config,
                          State& state,
                          Solid_state const& solid_state,
                          Temp_data& temp) {
    auto const particle_count = state.positions.size();
    dfsph_compute_alpha_denom_parts(particle_count,
                                    config.mass_per_particle,
                                    temp.alpha_denom_parts.data(),
                                    temp.neighbor_counts.data(),
                                    temp.neighbor_kernel_gradients.data());
    dfsph_accumulate_alpha_denom_parts_from_solids(
      particle_count,
      config.params.target_density,
      temp.alpha_denom_parts.data(),
      solid_state.volumes.data(),
      temp.solid_neighbor_counts.data(),
      temp.solid_neighbor_indices.data(),
      temp.solid_neighbor_kernel_gradients.data());
    dfsph_compute_alphas_from_parts(particle_count,
                                    temp.alphas.data(),
                                    temp.densities.data(),
                                    temp.alpha_denom_parts.data());
}

float dfsph_compute_divergence_error_kappas(float const dt,
                                            Simulation_config const& config,
                                            State& state,
                                            Solid_state const& solid_state,
                                            Temp_data& temp) {
    auto const particle_count = state.positions.size();
    dfsph_compute_density_dots(particle_count,
                               config.mass_per_particle,
                               temp.density_stars.data(),
                               state.velocities.data(),
                               temp.neighbor_counts.data(),
                               temp.neighbor_indices.data(),
                               temp.neighbor_kernel_gradients.data());
    dfsph_accumulate_density_dots_from_solids(
      particle_count,
      config.params.target_density,
      temp.density_stars.data(),
      state.velocities.data(),
      solid_state.velocities.data(),
      solid_state.volumes.data(),
      temp.solid_neighbor_counts.data(),
      temp.solid_neighbor_indices.data(),
      temp.solid_neighbor_kernel_gradients.data());

    auto const avg_density_dot = dfsph_average_density_dot(
      particle_count, config.params.target_density, temp.density_stars.data());

    dfsph_compute_kappas_from_density_dots(particle_count,
                                           dt,
                                           temp.divergence_kappas.data(),
                                           temp.density_stars.data(),
                                           temp.alphas.data());

    return avg_density_dot;
}

void dfsph_apply_kappas(float const dt,
                        float const* const kappas,
                        Simulation_config const& config,
                        State& state,
                        Solid_state const& solid_state,
                        Temp_data& temp) {
    auto const particle_count = state.positions.size();
    dfsph_apply_kappas_to_velocities(particle_count,
                                     dt,
                                     config.mass_per_particle,
                                     state.velocities.data(),
                                     kappas,
                                     temp.densities.data(),
                                     temp.neighbor_counts.data(),
                                     temp.neighbor_indices.data(),
                                     temp.neighbor_kernel_gradients.data());

    dfsph_apply_kappas_to_velocities_from_solids(
      particle_count,
      dt,
      config.params.target_density,
      state.velocities.data(),
      kappas,
      temp.densities.data(),
      solid_state.volumes.data(),
      temp.solid_neighbor_counts.data(),
      temp.solid_neighbor_indices.data(),
      temp.solid_neighbor_kernel_gradients.data());
}

void dfsph_compute_densities(float* const densities,
                             Simulation_config const& config,
                             State& state,
                             Solid_state const& solid_state,
                             Temp_data& temp) {
    auto const count = state.positions.size();
    compute_densities(count,
                      config.mass_per_particle,
                      config.params.support,
                      densities,
                      temp.neighbor_counts.data(),
                      temp.neighbor_kernels.data());

    accumulate_density_from_solids(count,
                                   config.params.target_density,
                                   densities,
                                   solid_state.volumes.data(),
                                   temp.solid_neighbor_counts.data(),
                                   temp.solid_neighbor_indices.data(),
                                   temp.solid_neighbor_kernels.data());
}

void dfsph_correct_divergence_error(float const dt,
                                    Simulation_config const& config,
                                    State& state,
                                    Solid_state const& solid_state,
                                    Temp_data& temp) {
    // Warm-start
    dfsph_apply_kappas(
      dt, temp.divergence_kappas.data(), config, state, solid_state, temp);
    for (int iter = 0; iter < config.params.dfsph.max_correction_iterations;
         ++iter) {
        auto const average_density_dot = dfsph_compute_divergence_error_kappas(
          dt, config, state, solid_state, temp);
        fmt::print("Average Density Dot: {}\n", average_density_dot);
        dfsph_apply_kappas(
          dt, temp.divergence_kappas.data(), config, state, solid_state, temp);
        if (iter > 0 && average_density_dot <=
                          config.params.dfsph.average_density_dot_tolerance) {
            fmt::print(
              "Correct Divergence Error: Exiting after {} iterations\n", iter);
            return;
        }
    }
    fmt::print("Correct Divergence Error: Reached max iterations: {}\n",
               config.params.dfsph.max_correction_iterations);
}

void dfsph_compute_density_stars_from_densities_and_density_dots(
  size_t const particle_count,
  float const dt,
  float* const density_stars,
  float const* const density_dots,
  float const* const densities) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        density_stars[particle_index] =
          densities[particle_index] + dt * density_dots[particle_index];
    });
}

void dfsph_compute_density_stars(float const dt,
                                 Simulation_config const& config,
                                 State& state,
                                 Solid_state const& solid_state,
                                 Temp_data& temp) {
    auto const particle_count = state.positions.size();
    dfsph_compute_density_dots(particle_count,
                               config.mass_per_particle,
                               temp.density_stars.data(),
                               state.velocities.data(),
                               temp.neighbor_counts.data(),
                               temp.neighbor_indices.data(),
                               temp.neighbor_kernel_gradients.data());
    dfsph_accumulate_density_dots_from_solids(
      particle_count,
      config.params.target_density,
      temp.density_stars.data(),
      state.velocities.data(),
      solid_state.velocities.data(),
      solid_state.volumes.data(),
      temp.solid_neighbor_counts.data(),
      temp.solid_neighbor_indices.data(),
      temp.solid_neighbor_kernel_gradients.data());

    dfsph_compute_density_stars_from_densities_and_density_dots(
      particle_count,
      dt,
      temp.density_stars.data(),
      temp.density_stars.data(),
      temp.densities.data());
}

void dfsph_correct_density_error(float const dt,
                                 Simulation_config const& config,
                                 State& state,
                                 Solid_state const& solid_state,
                                 Temp_data& temp) {
    auto const count = state.positions.size();
    // Warm start
    dfsph_apply_kappas(
      dt, temp.density_kappas.data(), config, state, solid_state, temp);
    for (int iter = 0; iter < config.params.dfsph.max_correction_iterations;
         ++iter) {
        dfsph_compute_density_stars(dt, config, state, solid_state, temp);
        auto const avg_density_star = dfsph_average_density(
          count, config.params.target_density, temp.density_stars.data());
        fmt::print("Average density star: {}\n", avg_density_star);

        dfsph_compute_kappas_from_density_errors(count,
                                                 dt,
                                                 config.params.target_density,
                                                 temp.density_kappas.data(),
                                                 temp.density_stars.data(),
                                                 temp.alphas.data());
        dfsph_apply_kappas(
          dt, temp.density_kappas.data(), config, state, solid_state, temp);
        if (iter > 1 &&
            (avg_density_star - config.params.target_density) <=
              config.params.dfsph.average_density_star_error_tolerance) {
            fmt::print("Correct Density Error: Exiting after {} iterations\n",
                       iter);
            return;
        }
    }

    fmt::print("Correct Density Error: Reached max iterations: {}\n",
               config.params.dfsph.max_correction_iterations);
}

void dfsph_sub_step(float const dt,
                    Simulation_config const& config,
                    State& state,
                    Solid_state const& solid_state,
                    Temp_data& temp) {
    // This is the ordering from the paper, but it is hard because
    // it has a position update mid-way. I'm going to just rotate these
    // through the sub timestep.
    //--------------------------------------------------------------------------
    // dfsph_compute_all_external_forces(config, state, temp);
    // dfsph_integrate_velocity_from_non_pressure_forces(dt, config, state,
    // solid_state, temp); dfsph_correct_density_error(dt, config, state,
    // solid_state, temp); dfsph_integrate_positions(dt, config, state,
    // solid_state, temp);
    // // TODO: MOVE SOLIDS HERE
    // compute_all_neighbhorhoods(config, state, solid_state, temp);
    // dfsph_compute_densities(
    //   temp.densities.data(), config, state, solid_state, temp);
    // dfsph_compute_alphas(config, state, solid_state, temp);
    // dfsph_correct_divergence_error(dt, config, state, solid_state, temp);

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);
    dfsph_compute_densities(
      temp.densities.data(), config, state, solid_state, temp);
    dfsph_compute_alphas(config, state, solid_state, temp);
    // HACK the 1.0f should be dt
    dfsph_correct_divergence_error(1.0f, config, state, solid_state, temp);
    compute_all_external_forces(config, state, temp);
    dfsph_integrate_velocity_from_non_pressure_forces(
      dt, config, state, solid_state, temp);
    // HACK the 1.0f should be dt
    dfsph_correct_density_error(1.0f, config, state, solid_state, temp);
    dfsph_integrate_positions(dt, config, state, solid_state, temp);
}

void dfsph_resize_and_init_temp_arrays(size_t const particle_count,
                                       Temp_data& temp) {
    temp.external_forces.resize(particle_count);
    temp.alpha_denom_parts.resize(particle_count);
    temp.neighbor_counts.resize(particle_count);
    temp.neighbor_indices.resize(particle_count);
    temp.neighbor_distances.resize(particle_count);
    temp.neighbor_vectors_to.resize(particle_count);
    temp.neighbor_kernels.resize(particle_count);
    temp.neighbor_kernel_gradients.resize(particle_count);
    temp.solid_neighbor_counts.resize(particle_count);
    temp.solid_neighbor_indices.resize(particle_count);
    temp.solid_neighbor_distances.resize(particle_count);
    temp.solid_neighbor_vectors_to.resize(particle_count);
    temp.solid_neighbor_kernels.resize(particle_count);
    temp.solid_neighbor_kernel_gradients.resize(particle_count);
    temp.alphas.resize(particle_count);
    temp.densities.resize(particle_count);
    temp.divergence_kappas.resize(particle_count);
    temp.density_kappas.resize(particle_count);
    temp.density_stars.resize(particle_count);

    fill_array(particle_count, 0.0f, temp.divergence_kappas.data());
    fill_array(particle_count, 0.0f, temp.density_kappas.data());
}

State dfsph_simulation_step(Simulation_config const& config,
                            State&& state,
                            const Solid_state& solid_state,
                            Temp_data& temp) {
    flicks const min_time_step = config.params.time_per_step / 1000;

    auto const particle_count = state.positions.size();
    dfsph_resize_and_init_temp_arrays(particle_count, temp);

    int sub_steps = 0;
    flicks remaining_time_step = config.params.time_per_step;
    while (remaining_time_step.count() > 0) {
        auto const cfl_step = dfsph_cfl_maximum_time_step(
          particle_count, config.params.support, state.velocities.data());
        if (cfl_step < min_time_step) {
            fmt::print(stderr,
                       "WARNING: CFL max time step is too small: {}\n",
                       to_seconds(cfl_step));
        }
        auto sub_time_step =
          std::clamp(cfl_step, min_time_step, remaining_time_step);
        if ((remaining_time_step - sub_time_step) < min_time_step) {
            sub_time_step = flicks{(remaining_time_step.count() + 1) / 2};
        }
        dfsph_sub_step(
          to_seconds(sub_time_step), config, state, solid_state, temp);

        remaining_time_step -= sub_time_step;
        ++sub_steps;

        // HACK
        break;
    }

    state.colors.resize(particle_count);
    compute_colors(particle_count,
                   config.params.target_density,
                   state.colors.data(),
                   temp.densities.data());

    fmt::print("DFSPH frame complete, sub_steps: {}\n", sub_steps);

    return std::move(state);
}

}  // namespace emerald::sph2d_box
