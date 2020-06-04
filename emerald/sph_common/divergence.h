#pragma once

#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

namespace emerald::sph_common {

void compute_divergences(size_t const particle_count,
                         float const target_density,
                         float* const divergences,
                         float const* const fluid_volumes,
                         V2f const* const fluid_velocities,
                         Neighborhood_pointers const fluid_neighborhood,
                         float const* const solid_volumes,
                         V2f const* const solid_velocities,
                         Neighborhood_pointers const solid_neighborhood);

}  // namespace emerald::sph_common
