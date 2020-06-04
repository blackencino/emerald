#pragma once

#include <cstdint>

namespace emerald::sph_common {

void compute_constant_volumes(size_t const particle_count,
                              float const target_density,
                              float const mass_per_particle,
                              float* const volumes);
}  // namespace emerald::sph_common
