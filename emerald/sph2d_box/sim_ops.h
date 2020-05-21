#pragma once

#include <emerald/sph2d_box/tag.h>
#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

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

void compute_densities(size_t const particle_count,
                       float const mass_per_particle,
                       float const support,
                       float* const densities,
                       uint8_t const* const neighbor_counts,
                       Neighbor_values<float> const* const neighbor_kernels);

void update_pressures(size_t const particle_count,
                      float const target_density,
                      float const pressure_correction_denom,
                      float* const pressures,
                      float const* const densities);

void accumulate_pressure_forces(
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

float max_density_error(size_t const particle_count,
                        float const target_density,
                        float const* const densities);

void compute_colors(size_t const particle_count,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities);

}  // namespace emerald::sph2d_box
