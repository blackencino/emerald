#include <emerald/sph_common/volume.h>

#include <emerald/sph_common/common.h>

namespace emerald::sph_common {

void compute_constant_volumes(size_t const particle_count,
                              float const target_density,
                              float const mass_per_particle,
                              float* const volumes) {
    // mass = volume * density,
    // volume = mass / density
    fill_array(particle_count, mass_per_particle / target_density, volumes);
}

}  // namespace emerald::sph_common