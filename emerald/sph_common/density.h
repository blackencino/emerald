#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>

namespace emerald::sph_common {

void compute_densities(size_t const particle_count,
                       float const support,
                       float const target_density,
                       float* const densities,
                       float const* const fluid_volumes,
                       Neighborhood_pointers const fluid_neighborhood,
                       float const* const solid_volumes,
                       Neighborhood_pointers const solid_neighborhood);

void predict_density_stars_from_divergences(size_t const particle_count,
                                            float const dt,
                                            float* const density_stars,
                                            float const* const densities,
                                            float const* const divergences);

void predict_density_stars(size_t const particle_count,
                           float const dt,
                           float const target_density,
                           float* const density_stars,
                           float const* const densities,
                           float const* const fluid_volumes,
                           V2f const* const fluid_velocities,
                           Neighborhood_pointers const fluid_neighborhood,
                           float const* const solid_volumes,
                           V2f const* const solid_velocities,
                           Neighborhood_pointers const solid_neighborhood);

}  // namespace emerald::sph_common