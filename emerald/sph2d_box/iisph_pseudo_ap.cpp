#include <emerald/sph2d_box/iisph_pseudo_ap.h>

#include <emerald/sph2d_box/adaptive_time_step.h>
#include <emerald/sph2d_box/iisph_pseudo_ap_ops.h>
#include <emerald/sph2d_box/neighborhoods_from_state.h>
#include <emerald/sph_common/cfl.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/density.h>
#include <emerald/sph_common/dynamics.h>
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
// The entire pressure solver portion can be written in terms of displacements
// and have a fictional timestep of 1. This eliminates a lot of multiplication
// and division by dt and dt^2, and brings the IISPH formulation more in line
// with the "position-based dynamics" methodology that is popular. This
// solver implements the IISPH pressure-acceleration formulation from the
// 2019 paper, with a fictitious timestep of 1 during the pressure solver
// portion. Accordingly, time integration at the end is handled in terms of
// displacements.
//
// This method bears the named "IISPH PSEUDO AP"
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

static void iisph_pseudo_ap_resize_temp_arrays(size_t const particle_count,
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

static void iisph_pseudo_ap_sub_step(float const global_time,
                                     float const dt,
                                     Simulation_config const& config,
                                     State& state,
                                     Solid_state const& solid_state,
                                     Temp_data& temp,
                                     User_forces_function const& user_forces) {
    auto const particle_count = state.positions.size();

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_neighborhood_kernels(config, temp);
    compute_all_external_forces(
      global_time, dt, config, state, solid_state, temp, user_forces);

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

State iisph_pseudo_ap_simulation_step(flicks const global_time,
                                      Simulation_config const& config,
                                      State&& state,
                                      Solid_state const& solid_state,
                                      Temp_data& temp,
                                      User_forces_function const& user_forces,
                                      User_colors_function const& user_colors) {
    return std_adaptive_time_step("IISPH PSEUDO AP",
                                  iisph_pseudo_ap_resize_temp_arrays,
                                  iisph_pseudo_ap_sub_step,
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
