#include <emerald/sph_common/divergence.h>

namespace emerald::sph_common {

static void compute_divergences_partial(
  size_t const particle_count,
  bool const boundary,
  float const target_density,
  float* const divergences,
  V2f const* const self_velocities,
  float const* const neighbor_volumes,
  V2f const* const neighbor_velocities,
  Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            if (!boundary) { divergences[particle_index] = 0.0f; }
            return;
        }

        auto const self_velocity = self_velocities[particle_index];

        float divergence = 0.0f;

        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            auto const neighbor_mass =
              target_density * neighbor_volumes[neighbor_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];
            auto const neighbor_velocity =
              neighbor_velocities[neighbor_particle_index];

            divergence +=
              neighbor_mass * ((self_velocity - neighbor_velocity).dot(grad_w));
        }

        if (!boundary) {
            divergences[particle_index] = divergence;
        } else {
            divergences[particle_index] += divergence;
        }
    });
}

void compute_divergences(size_t const particle_count,
                         float const target_density,
                         float* const divergences,
                         float const* const fluid_volumes,
                         V2f const* const fluid_velocities,
                         Neighborhood_pointers const fluid_neighborhood,
                         float const* const solid_volumes,
                         V2f const* const solid_velocities,
                         Neighborhood_pointers const solid_neighborhood) {
    compute_divergences_partial(particle_count,
                                false,
                                target_density,
                                divergences,
                                fluid_velocities,
                                fluid_volumes,
                                fluid_velocities,
                                fluid_neighborhood);

    compute_divergences_partial(particle_count,
                                true,
                                target_density,
                                divergences,
                                fluid_velocities,
                                solid_volumes,
                                solid_velocities,
                                solid_neighborhood);
}

}  // namespace emerald::sph_common
