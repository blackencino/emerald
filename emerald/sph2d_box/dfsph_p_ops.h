#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>
#include <utility>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

void dfsph_p_compute_shared_coeffs(
  size_t const particle_count,
  float const target_density,
  float* const shared_coeffs,
  float const* const densities,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood);

void dfsph_p_compute_density_pseudo_pressures(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const density_pseudo_pressures,
  float const* const density_stars,
  float const* const shared_coeffs);

void dfsph_p_compute_divergence_pseudo_pressures(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const divergence_pseudo_pressures,
  float const* const divergences,
  float const* const shared_coeffs);

void dfsph_p_integrate_pseudo_pressures(
  size_t const particle_count,
  float const dt,
  float const target_density,
  V2f* const velocity_stars,
  V2f* const pressure_accelerations,
  float const* const pseudo_pressures,
  float const* const densities,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood);

}  // namespace emerald::sph2d_box
