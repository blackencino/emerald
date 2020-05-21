#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

size_t estimate_solid_box_emission_count(Box2f const box, float const R);

size_t emit_solid_box(Box2f const box,
                      float const R,
                      size_t const max_count,
                      V2f* const positions);

void compute_volumes(size_t const particle_count,
                     float const support,
                     float* const volumes,
                     uint8_t const* const neighbor_counts,
                     Neighbor_values<float> const* const neighbor_kernels);

Solid_state compute_neighbor_data_and_volumes(Parameters const& params,
                                              Solid_state&& solid_state);

Solid_state world_walls_initial_solid_state(Parameters const& params);

// Stuff related to solids
void accumulate_density_from_solids(
  size_t const particle_count,
  float const target_density,
  float* const densities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<float> const* const solid_neighbor_kernels);

void accumulate_pressure_forces_from_solids(
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