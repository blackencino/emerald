#include <emerald/sph_common/density.h>

#include <emerald/sph_common/divergence.h>
#include <emerald/sph_common/kernels.h>

namespace emerald::sph_common {

//------------------------------------------------------------------------------
// Densities
static void compute_densities_partial(
  size_t const particle_count,
  bool const boundary,
  float const support,
  float const target_density,
  float* const densities,
  float const* const self_volumes,
  float const* const neighbor_volumes,
  Neighborhood_pointers const neighborhood) {
    auto const w0 = kernels::W(0.0f, support);
    for_each_iota(particle_count, [=](auto const particle_index) {
        float density = 0.0f;
        if (!boundary) {
            density = self_volumes[particle_index] * target_density * w0;
        }
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            if (!boundary) { densities[particle_index] = density; }
            return;
        }

        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernels = neighborhood.kernels[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            auto const mass =
              target_density * neighbor_volumes[neighbor_particle_index];
            density += nbhd_kernels[j] * mass;
        }

        if (boundary) {
            densities[particle_index] += density;
        } else {
            densities[particle_index] = density;
        }
    });
}

void compute_densities(size_t const particle_count,
                       float const support,
                       float const target_density,
                       float* const densities,
                       float const* const fluid_volumes,
                       Neighborhood_pointers const fluid_neighborhood,
                       float const* const solid_volumes,
                       Neighborhood_pointers const solid_neighborhood) {
    compute_densities_partial(particle_count,
                              false,
                              support,
                              target_density,
                              densities,
                              fluid_volumes,
                              fluid_volumes,
                              fluid_neighborhood);

    compute_densities_partial(particle_count,
                              true,
                              support,
                              target_density,
                              densities,
                              fluid_volumes,
                              solid_volumes,
                              solid_neighborhood);
}

void predict_density_stars_from_divergences(size_t const particle_count,
                                            float const dt,
                                            float* const density_stars,
                                            float const* const densities,
                                            float const* const divergences) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        density_stars[particle_index] = std::max(
          0.0f, densities[particle_index] + (dt * divergences[particle_index]));
    });
}

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
                           Neighborhood_pointers const solid_neighborhood) {
    compute_divergences(particle_count,
                        target_density,
                        density_stars,
                        fluid_volumes,
                        fluid_velocities,
                        fluid_neighborhood,
                        solid_volumes,
                        solid_velocities,
                        solid_neighborhood);

    predict_density_stars_from_divergences(
      particle_count, dt, density_stars, densities, density_stars);
}

}  // namespace emerald::sph_common
