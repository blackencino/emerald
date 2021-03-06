#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>
#include <utility>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

void iisph_compute_diis(size_t const particle_count,
                        float const dt,
                        float const target_density,
                        V2f* const diis,

                        float const* const self_densities,

                        float const* const fluid_volumes,
                        Neighborhood_pointers const fluid_neighborhood,

                        float const* const solid_volumes,
                        Neighborhood_pointers const solid_neighborhood);

void iisph_compute_aiis_and_density_stars(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const aiis,
  float* const density_stars,

  V2f const* const diis,
  float const* const densities,

  float const* const fluid_volumes,
  V2f const* const fluid_velocities,
  Neighborhood_pointers const fluid_neighborhood,

  float const* const solid_volumes,
  V2f const* const solid_velocities,
  Neighborhood_pointers const solid_neighborhood);

void iisph_compute_sum_dij_pjs(size_t const particle_count,
                               float const dt,
                               float const target_density,
                               V2f* const sum_dij_pjs,
                               float const* const volumes,
                               float const* const densities,
                               float const* const pressures,
                               Neighborhood_pointers const neighborhood);

void iisph_compute_betas(size_t const particle_count,
                         float dt,
                         float const target_density,
                         float* const betas,
                         float const* const densities,
                         float const* const pressures,
                         V2f const* const diis,
                         V2f const* const sum_dij_pjs,
                         float const* const fluid_volumes,
                         Neighborhood_pointers const fluid_neighborhood,
                         float const* const solid_volumes,
                         Neighborhood_pointers const solid_neighborhood);

std::pair<float, float> iisph_compute_new_pressures(
  size_t const particle_count,
  float const omega,
  float const target_density,
  float* const betas_in_new_pressures_out,
  float const* const old_pressures,
  float const* const alphas,
  float const* const density_stars);

}  // namespace emerald::sph2d_box
