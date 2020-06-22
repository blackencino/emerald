#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

void pcisph_compute_densities(
  size_t const particle_count,
  float const mass_per_particle,
  float const support,
  float* const densities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_kernels);

void pcisph_accumulate_density_from_solids(
  size_t const particle_count,
  float const target_density,
  float* const densities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<float> const* const solid_neighbor_kernels);

void pcisph_update_pressures(size_t const particle_count,
                             float const target_density,
                             float const pressure_correction_denom,
                             float* const pressures,
                             float const* const densities);

void pcisph_accumulate_pressure_forces(
  size_t const particle_count,
  float const mass_per_particle,
  float const support,
  float const viscosity,
  V2f* const pressure_forces,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<float> const* const neighbor_distances,
  Neighbor_values<V2f> const* const neighbor_vectors_to,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients,
  V2f const* const velocities,
  float const* const pressures,
  float const* const densities);

void pcisph_accumulate_pressure_forces_from_solids(
  size_t const particle_count,
  float const mass_per_particle,
  float const target_density,
  V2f* const pressure_forces,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients,
  float const* const pressures,
  float const* const densities);

}  // namespace emerald::sph2d_box
