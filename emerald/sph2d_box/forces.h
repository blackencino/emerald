#pragma once

#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/solids.h>
#include <emerald/sph2d_box/state.h>
#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <functional>

namespace emerald::sph2d_box {

using User_forces_function =
  std::function<void(size_t const,        // count
                     float const,         // time (in seconds)
                     float const,         // dt
                     float const,         // mass per particle
                     float const,         // radius per particle
                     V2f* const,          // forces
                     V2f const* const,    // positions
                     V2f const* const)>;  // velocities

void compute_all_external_forces(float const global_time_in_seconds,
                                 float const dt,
                                 Simulation_config const& config,
                                 State const& state,
                                 Solid_state const& solid_state,
                                 Temp_data& temp,
                                 User_forces_function const& user_forces);

// These should no longer be used because of user forces, but just in case:

User_forces_function default_gravity_forces(float const gravity);

void accumulate_gravity_forces(size_t const particle_count,
                               float const mass_per_particle,
                               float const gravity,
                               V2f* const forces);

void accumulate_constant_pole_attraction_forces(size_t const particle_count,
                                                float const magnitude,
                                                V2f const pole,
                                                V2f* const forces,
                                                V2f const* const positions);

void accumulate_simple_drag_forces(size_t const particle_count,
                                   float const magnitude,
                                   float const particle_diameter,
                                   V2f* const forces,
                                   V2f const* const velocities);

void accumulate_anti_coupling_repulsive_forces(
  size_t const particle_count,
  float const max_distance,
  float const force_magnitude,
  V2f* const forces,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_distances);

}  // namespace emerald::sph2d_box
