#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>
#include <utility>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

void iisph_ap_density_stars_and_diagonals(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const density_stars,
  float* const diagonals,

  float const* const densities,

  float const* const fluid_volumes,
  V2f const* const fluid_velocities,
  Neighborhood_pointers const fluid_neighborhood,

  float const* const solid_volumes,
  V2f const* const solid_velocities,
  Neighborhood_pointers const solid_neighborhood);

std::pair<float, float> iisph_ap_iterate_pressures_in_place(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float const omega,
  float* const pressures,
  float const* const density_stars,
  V2f const* const pressure_accelerations,
  float const* const diagonals,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood);

}  // namespace emerald::sph2d_box
