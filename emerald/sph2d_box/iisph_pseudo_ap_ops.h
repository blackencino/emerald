#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>
#include <utility>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

void iisph_pseudo_ap_density_stars_and_pseudo_diagonals(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const density_stars,
  float* const pseudo_diagonals,

  float const* const densities,

  float const* const fluid_volumes,
  V2f const* const fluid_velocities,
  Neighborhood_pointers const fluid_neighborhood,

  float const* const solid_volumes,
  V2f const* const solid_velocities,
  Neighborhood_pointers const solid_neighborhood);

void iisph_pseudo_ap_compute_pseudo_pressure_displacements(
  size_t const particle_count,
  float const target_density,

  V2f* const pseudo_pressure_displacements,

  float const* const pseudo_pressures,
  float const* const densities,

  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood);

std::pair<float, float> iisph_pseudo_ap_iterate_pseudo_pressures_in_place(
  size_t const particle_count,
  float const target_density,
  float const omega,
  float* const pseudo_pressures,
  float const* const density_stars,
  V2f const* const pseudo_pressure_displacements,
  float const* const pseudo_diagonals,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood);

//------------------------------------------------------------------------------
// Don't replace this with the common/dynamics integration! PSEUDO AP
// uses the displacements to compute the velocity change, which is different.
void iisph_pseudo_ap_integrate_velocities_and_positions_in_place(
  size_t const particle_count,
  float const dt,
  V2f* const velocities,
  V2f* const positions,
  V2f const* const pseudo_pressure_displacements);

}  // namespace emerald::sph2d_box
