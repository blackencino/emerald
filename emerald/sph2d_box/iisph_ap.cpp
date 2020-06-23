#include <emerald/sph2d_box/iisph_ap.h>

#include <emerald/sph2d_box/adaptive_time_step.h>
#include <emerald/sph2d_box/iisph_ap_ops.h>
#include <emerald/sph2d_box/neighborhoods_from_state.h>
#include <emerald/sph_common/cfl.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/density.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/pressure.h>
#include <emerald/sph_common/volume.h>
#include <emerald/util/assert.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>

#include <fmt/format.h>

#include <cmath>

namespace emerald::sph2d_box {

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// This solver is an implementation of the Implicit Incompressible SPH method
// originally published in:
// Implicit Incompressible SPH"
// by Markus Ihmsen, Jens Cornelis, Barbara Solenthaler, Christopher Horvath,
// and Matthias Teschner
// IEEE Transactions on Visualization and Computer Graphics, 2013
//
// That original paper was written in terms of forces, and a more elegant
// formulation written instead in terms of "acceleration due to pressure"
// was presented in:
//"Smoothed Particle Hydrodynamics Techniques for the Physics Based Simulation
// of Fluids and Solids", by Dan Koschier, Jan Bender,
// Barbara Solenthaler, and Matthias Teschner,
// Eurographics Proceedings 2019
//
// This solver implements the acceleration due to pressure IISPH formulation,
// hence the name "IISPH_AP".
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

static void iisph_ap_sub_step(float const global_time_in_seconds,
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
        compute_pressure_accelerations(particle_count,
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

        // fmt::print("\t Pressure iter: {}, error avg: {}, max: {}\n",
        //            iter,
        //            error_average,
        //            error_max);

        if (iter > 1 &&
            error_average <= config.params.iisph.error_average_threshold &&
            error_max <= config.params.iisph.error_max_threshold) {
            // fmt::print(
            //   "IISPH AP pressure solve early exit. err avg: {}, max: {}\n",
            //   error_average,
            //   error_max);
            break;
        }
    }
    // fmt::print("IISPH AP pressure iters: {}\n", iter);

    compute_pressure_accelerations(particle_count,
                                   config.params.target_density,
                                   temp.pressure_accelerations.data(),
                                   temp.pressures.data(),
                                   temp.densities.data(),
                                   temp.fluid_volumes.data(),
                                   temp.neighborhood.pointers(),
                                   solid_state.volumes.data(),
                                   temp.solid_neighborhood.pointers());

    integrate_velocities_and_positions_in_place(
      particle_count,
      dt,
      state.velocities.data(),
      state.positions.data(),
      temp.pressure_accelerations.data());
}

static void iisph_ap_resize_temp_arrays(size_t const particle_count,
                                        Temp_data& temp) {
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

State iisph_ap_simulation_step(flicks const global_time,
                               Simulation_config const& config,
                               State&& state,
                               Solid_state const& solid_state,
                               Temp_data& temp,
                               User_forces_function const& user_forces,
                               User_colors_function const& user_colors) {
    return std_adaptive_time_step("IISPH AP",
                                  iisph_ap_resize_temp_arrays,
                                  iisph_ap_sub_step,
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