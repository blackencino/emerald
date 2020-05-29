#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

float dfsph_average_density(size_t const particle_count,
                            float const target_density,
                            float const* const densities);

float dfsph_average_density_dot(size_t const particle_count,
                                float const target_density,
                                float const* const density_dots);

void dfsph_compute_alpha_denom_parts(
  size_t const particle_count,
  float const mass_per_particle,
  V3f* const alpha_denom_parts,
  uint8_t const* const neighbor_counts,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients);

void dfsph_accumulate_alpha_denom_parts_from_solids(
  size_t const particle_count,
  float const target_density,
  V3f* const alpha_denom_parts,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients);

void dfsph_compute_alphas_from_parts(size_t const particle_count,
                                     float* const alphas,
                                     float const* const densities,
                                     V3f const* const alpha_denom_parts);

void dfsph_compute_density_dots(
  size_t const particle_count,
  float const mass_per_particle,
  float* const density_dots,
  V2f const* const velocities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients);

void dfsph_accumulate_density_dots_from_solids(
  size_t const particle_count,
  float const target_density,
  float* const density_dots,
  V2f const* const velocities,
  V2f const* const solid_velocities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients);

void dfsph_compute_kappas_from_density_dots(size_t const particle_count,
                                            float const dt,
                                            float* const kappas,
                                            float const* const density_dots,
                                            float const* const alphas);

void dfsph_compute_kappas_from_density_errors(size_t const particle_count,
                                              float const dt,
                                              float const target_density,
                                              float* const kappas,
                                              float const* const densities,
                                              float const* const alphas);

void dfsph_apply_kappas_to_velocities(
  size_t const particle_count,
  float const dt,
  float const mass_per_particle,
  V2f* const velocities,
  float const* const kappas,
  float const* const densities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients);

void dfsph_apply_kappas_to_velocities_from_solids(
  size_t const particle_count,
  float const dt,
  float const target_density,
  V2f* const velocities,
  float const* const kappas,
  float const* const densities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients);

}  // namespace emerald::sph2d_box
