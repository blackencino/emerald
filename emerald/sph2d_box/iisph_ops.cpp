#include <emerald/sph2d_box/iisph_ops.h>

namespace emerald::sph2d_box {

void compute_diis_times_squared_densities(
  size_t const particle_count,
  float const dt,
  float const mass_per_particle,
  V2f* const diis_times_squared_densities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) {
            diis_times_squared_densities[particle_index] = V2f{0.0f, 0.0f};
            return;
        }

        V2f m_grad_sum{0.0f, 0.0f};
        auto const& nbhd_kernel_gradients =
          neighbor_kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            m_grad_sum += mass_per_particle * nbhd_kernel_gradients[j];
        }

        diis_times_squared_densities[particle_index] = -sqr(dt) * m_grad_sum;
    });
}

void accumulate_diis_times_squared_densities_from_solids(
  size_t const particle_count,
  float const dt,
  float const target_density,
  V2f* const diis_times_squared_densities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = solid_neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        V2f m_grad_sum{0.0f, 0.0f};
        auto const& nbhd_indices = solid_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          solid_neighbor_kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_volume = solid_volumes[other_particle_index];
            m_grad_sum +=
              target_density * other_volume * nbhd_kernel_gradients[j];
        }

        diis_times_squared_densities[particle_index] += -sqr(dt) * m_grad_sum;
    });
}

void divide_by_sqr_densities_to_get_diis(size_t const particle_count,
                                         V2f* const diis,
                                         float const* const densities) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const numer = diis[particle_index];
        auto const denom = sqr(densities[particle_index]);
        diis[particle_index] =
          safe_divide(numer, denom).value_or(V2f{0.0f, 0.0f});
    });
}

void compute_aiis_and_density_stars(size_t const particle_count,
                                    float const dt,
                                    float const mass_per_particle,
                                    float* const aiis,
                                    float* const density_stars,
                                    uint8_t const* const neighbor_counts,
                                    Neighbor_values<V2f> const* const neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {



    });
}


}  // namespace emerald::sph2d_box