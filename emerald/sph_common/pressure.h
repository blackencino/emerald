#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

namespace emerald::sph_common {

void apply_pressures_in_place(size_t const particle_count,
                              float const dt,
                              float const target_density,
                              V2f* const velocities,
                              float const* const densities,
                              float const* const pressures,
                              float const* const fluid_volumes,
                              Neighborhood_pointers const fluid_neighborhood,
                              float const* const solid_volumes,
                              Neighborhood_pointers const solid_neighborhood);

void compute_pressure_accelerations(
  size_t const particle_count,
  float const target_density,
  V2f* const pressure_accelerations,
  float const* const pressures,
  float const* const densities,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood);

}  // namespace emerald::sph_common
