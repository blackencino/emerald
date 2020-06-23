#include <emerald/sph2d_box/dfsph_p.h>

#include <emerald/sph2d_box/adaptive_time_step.h>
#include <emerald/sph2d_box/dfsph_p_ops.h>
#include <emerald/sph2d_box/neighborhoods_from_state.h>
#include <emerald/sph_common/cfl.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/density.h>
#include <emerald/sph_common/divergence.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/pressure.h>
#include <emerald/sph_common/volume.h>
#include <emerald/util/assert.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>
#include <emerald/util/safe_divide.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// This solver is an implementation of the Divergence Free SPH method originally
// published in:
// "Divergence-Free SPH for Incompressible and Viscous Fluids"
// by Jan Bender and Dan Koschier,
// IEEE Transactions on Visualization and Computer Graphics, 2016
// The method described in this paper involves computing "stiffness"
// coefficients, which the paper uses the greek letter "kappa" for. These
// stiffnesses can be interpreted as pressures, and then normal pressure
// solver tools brought to bear.
//
// The magnificent paper,
// "Smoothed Particle Hydrodynamics Techniques for the Physics Based Simulation
// of Fluids and Solids", by Dan Koschier, Jan Bender,
// Barbara Solenthaler, and Matthias Teschner,
// Eurographics Proceedings 2019
// contains a reformulation of
// DFSPH written in terms of pressure instead of stiffness, and the code
// here uses that formulation, hence the prefix DFSPH_P.
//
// We've made one other change here, in order to adapt to our simulation
// framework. The DFSPH paper and its reformulation above has a weird split
// timestep, where the position update takes place in the middle of the
// simulation step. This causes several problems - firstly, it requires
// a "pre-init" step before the first iteration. Secondly, it requires that
// any updates of external state, such as rigid body positions, be done
// internally, which is problematic. It also makes emission and deletion of
// new particles really challenging, requiring a whole bunch of assumptions
// that entangle the solver. So, as noted in the substep below, we rotate
// the steps of the sub-step such that position update is last, and
// neighborhood computation is first. We've noticed no discernable problems
// with this approach.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


static void dfsph_p_correct_density_error(float const dt,
                                          Simulation_config const& config,
                                          State& state,
                                          Solid_state const& solid_state,
                                          Temp_data& temp) {
    auto const particle_count = state.positions.size();

    // Warm start
    apply_pressures_in_place(particle_count,
                             dt,
                             config.params.target_density,
                             state.velocities.data(),
                             temp.densities.data(),
                             temp.density_kappas.data(),
                             temp.fluid_volumes.data(),
                             temp.neighborhood.pointers(),
                             solid_state.volumes.data(),
                             temp.solid_neighborhood.pointers());

    for (int iter = 0; iter < config.params.dfsph.max_correction_iterations;
         ++iter) {
        predict_density_stars(particle_count,
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

        apply_pressures_in_place(particle_count,
                                 dt,
                                 config.params.target_density,
                                 state.velocities.data(),
                                 temp.densities.data(),
                                 temp.density_kappas.data(),
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

static void dfsph_p_correct_divergence_error(float const dt,
                                             Simulation_config const& config,
                                             State& state,
                                             Solid_state const& solid_state,
                                             Temp_data& temp) {
    auto const particle_count = state.positions.size();

    // Warm start
    apply_pressures_in_place(particle_count,
                             dt,
                             config.params.target_density,
                             state.velocities.data(),
                             temp.densities.data(),
                             temp.divergence_kappas.data(),
                             temp.fluid_volumes.data(),
                             temp.neighborhood.pointers(),
                             solid_state.volumes.data(),
                             temp.solid_neighborhood.pointers());

    for (int iter = 0; iter < config.params.dfsph.max_correction_iterations;
         ++iter) {
        compute_divergences(particle_count,
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

        apply_pressures_in_place(particle_count,
                                 dt,
                                 config.params.target_density,
                                 state.velocities.data(),
                                 temp.densities.data(),
                                 temp.divergence_kappas.data(),
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

static void dfsph_p_resize_and_init_temp_arrays(size_t const particle_count,
                                                Temp_data& temp) {
    temp.external_forces.resize(particle_count);
    temp.alphas.resize(particle_count);
    temp.neighborhood.resize(particle_count);
    temp.solid_neighborhood.resize(particle_count);
    temp.densities.resize(particle_count);
    temp.divergence_kappas.resize(particle_count, 0.0f);
    temp.density_kappas.resize(particle_count, 0.0f);
    temp.density_stars.resize(particle_count);
    temp.pressure_accelerations.resize(particle_count);
    temp.fluid_volumes.resize(particle_count);

    fill_array(particle_count, 0.0f, temp.divergence_kappas.data());
    fill_array(particle_count, 0.0f, temp.density_kappas.data());
}

//------------------------------------------------------------------------------
// NOT NEEDED because we rotated the timestep, see comment below.
// static void dfsph_p_init(Simulation_config const& config,
//                   State const& state,
//                   Solid_state const& solid_state,
//                   Temp_data& temp) {
//     auto const particle_count = state.positions.size();
//     dfsph_p_resize_and_init_temp_arrays(particle_count, temp);
//     compute_constant_volumes(particle_count,
//                              config.params.target_density,
//                              config.mass_per_particle,
//                              temp.fluid_volumes.data());
//     compute_all_neighbhorhoods(config, state, solid_state, temp);
//     compute_all_neighborhood_kernels(config, temp);

//     compute_densities(particle_count,
//                       config.params.support,
//                       config.params.target_density,
//                       temp.densities.data(),
//                       temp.fluid_volumes.data(),
//                       temp.neighborhood.pointers(),
//                       solid_state.volumes.data(),
//                       temp.solid_neighborhood.pointers());

//     dfsph_p_compute_shared_coeffs(particle_count,
//                                   config.params.target_density,
//                                   temp.alphas.data(),
//                                   temp.densities.data(),
//                                   temp.fluid_volumes.data(),
//                                   temp.neighborhood.pointers(),
//                                   solid_state.volumes.data(),
//                                   temp.solid_neighborhood.pointers());
// }

static void dfsph_p_sub_step(float const global_time_in_seconds,
                             float const dt,
                             Simulation_config const& config,
                             State& state,
                             Solid_state const& solid_state,
                             Temp_data& temp,
                             User_forces_function const& user_forces) {
    auto const particle_count = state.positions.size();

// The DFSPH paper calls for the ordering of substeps as below.
// However, this is weird. It places the position update in the
// middle of the timestep, where it is difficult to synchronize
// with another solver, such as a rigid body solver. Instead we rotate
// the timestep so that the position update comes last and the
// neighborhood computation comes first.
#if 0

    compute_all_external_forces(global_time_in_seconds,
                                dt,
                                config,
                                state,
                                solid_state,
                                temp,
                                user_forces);

    // External forces passed in
    integrate_velocities_in_place(particle_count,
                                  dt,
                                  config.params.target_density,
                                  state.velocities.data(),
                                  temp.fluid_volumes.data(),
                                  temp.external_forces.data());

    dfsph_p_correct_density_error(dt, config, state, solid_state, temp);

    integrate_positions_in_place(
      particle_count, dt, state.positions.data(), state.velocities.data());
    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);

    compute_densities(particle_count,
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

#else

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);

    compute_densities(particle_count,
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

    compute_all_external_forces(global_time_in_seconds + dt,
                                dt,
                                config,
                                state,
                                solid_state,
                                temp,
                                user_forces);

    // External forces passed in
    integrate_velocities_in_place(particle_count,
                                  dt,
                                  config.params.target_density,
                                  state.velocities.data(),
                                  temp.fluid_volumes.data(),
                                  temp.external_forces.data());

    dfsph_p_correct_density_error(dt, config, state, solid_state, temp);

    integrate_positions_in_place(
      particle_count, dt, state.positions.data(), state.velocities.data());

    // Velocity is updated in place.

#endif
}

State dfsph_p_simulation_step(flicks const global_time,
                              Simulation_config const& config,
                              State&& state,
                              Solid_state const& solid_state,
                              Temp_data& temp,
                              User_forces_function const& user_forces,
                              User_colors_function const& user_colors) {
    return std_adaptive_time_step("DFSPH P",
                                  dfsph_p_resize_and_init_temp_arrays,
                                  dfsph_p_sub_step,
                                  60,
                                  4,
                                  0.4f,
                                  global_time,
                                  config,
                                  std::move(state),
                                  solid_state,
                                  temp,
                                  user_forces,
                                  user_colors);
}

}  // namespace emerald::sph2d_box
