#include <emerald/sph2d_box/dfsph_ops.h>

namespace emerald::sph2d_box {

float dfsph_average_density(size_t const particle_count,
                            float const target_density,
                            float const* const densities) {
    return average_value(particle_count, 1000, target_density, densities);
}

void dfsph_compute_alpha_denom_parts(
  size_t const particle_count,
  float const mass_per_particle,
  V3f* const alpha_denom_parts,
  uint8_t const* const neighbor_counts,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const& nbhd_kernel_gradients =
          neighbor_kernel_gradients[particle_index];
        V2f mj_grad_w_sum{0.0f, 0.0f};
        float mj_grad_w_mag_sqr_sum = 0.0f;
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const mj_grad_w = mass_per_particle * nbhd_kernel_gradients[j];
            mj_grad_w_sum += mj_grad_w;
            mj_grad_w_mag_sqr_sum += mj_grad_w.dot(mj_grad_w);
        }

        alpha_denom_parts[particle_index] =
          V3f{mj_grad_w_sum[0].mj_grad_w_sum[1], mj_grad_w_mag_sqr_sum};
    });
}

void dfsph_accumulate_alpha_denom_parts_from_solids(
  size_t const particle_count,
  float const target_density,
  V3f* const alpha_denom_parts,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = solid_neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const& nbhd_indices = solid_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          solid_neighbor_kernel_gradients[particle_index];
        V2f phi_density0_grad_w_sum;
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_volume = solid_volumes[other_particle_index];
            phi_density0_grad_w_sum +=
              other_volume * target_density * nbhd_kernel_gradients[j];
        }

        alpha_denom_parts[particle_index] +=
          V3f{phi_density0_grad_w_sum[0].phi_density0_grad_w_sum[1], 0.0f};
    });
}

void dfsph_compute_alphas_from_parts(size_t const particle_count,
                                     float* const alphas,
                                     float const* const densities,
                                     V3f const* const alpha_denom_parts) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const denom_parts = alpha_denom_parts[particle_index];
        V2f const mj_grad_w_sum{parts[0], parts[1]};
        float const mj_grad_w_mag_sqr_sum = parts[2];

        auto const numer = densities[particle_index];
        auto const denom =
          mj_grad_w_sum.dot(mj_grad_w_sum) + mj_grad_w_mag_sqr_sum;

        alphas[particle_index] = safe_divide(numer, denom).value_or(0.0f);
    });
}

void dfsph_compute_density_dots(
  size_t const particle_count,
  float const mass_per_particle,
  float* const density_dots,
  V2f* const velocities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const velocity = velocities[particle_index];
        auto const& nbhd_indices = neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighbor_kernel_gradients[particle_index];
        float density_dot = 0.0f;
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_velocity = velocities[other_particle_index];
            density_dot +=
              mass_per_particle *
              ((velocity - other_velocity).dot(nbhd_kernel_gradients[j]));
        }

        density_dots[particle_index] = density_dot;
    });
}

void dfsph_accumulate_density_dots_from_solids(
  size_t const particle_count,
  float const target_density,
  float* const density_dots,
  V2f* const velocities,
  V2f* const solid_velocities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = solid_neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const velocity = velocities[particle_index];
        auto const& nbhd_indices = solid_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          solid_neighbor_kernel_gradients[particle_index];
        float density_dot = 0.0f;
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_velocity = solid_velocities[other_particle_index];
            auto const other_volume = solid_volumes[other_particle_index];

            density_dot +=
              target_density * other_volume *
              ((velocity - other_velocity).dot(nbhd_kernel_gradients[j]));
        }

        density_dots[particle_index] += density_dot;
    });
}

void dfsph_compute_kappas_from_density_dots(size_t const particle_count,
                                            float const dt,
                                            float* const kappas,
                                            float const* const density_dots,
                                            float const* const alphas) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const numer = alphas[particle_index] * density_dots[particle_index];
        kappas[particle_index] = safe_divide(numer, dt).value_or(0.0f);
    });
}

void dfsph_compute_kappas_from_density_errors(size_t const particle_count,
                                              float const dt,
                                              float const target_density,
                                              float* const kappas,
                                              float const* const densities,
                                              float const* const alphas) {
    auto const denom = sqr(dt);
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const numer =
          alphas[particle_index] * (densities[particle_index] - target_density);
        kappas[particle_index] = safe_divide(numer, denom).value_or(0.0f);
    });
}

void dfsph_apply_kappas_to_velocities(
  size_t const particle_count,
  float const dt,
  float const mass_per_particle,
  V2f* const velocities,
  float const* const kappas,
  float const* const densities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<uint8_t> const* const neighbor_indices,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const kappa_over_density =
          safe_divide(kappas[particle_index], densities[particle_index])
            .value_or(0.0f);
        V2f neg_acceleration_sum{0.0f, 0.0f};

        auto const& nbhd_indices = neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighbor_kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_kappa_over_density =
              safe_divide(kappas[other_particle_index],
                          densities[other_particle_index])
                .value_or(0.0f);
            neg_acceleration_sum +=
              mass_per_particle *
              (kappa_over_density + other_kappa_over_density) *
              nbhd_kernel_gradients[j];
        }

        velocities[particle_index] -= dt * neg_acceleration_sum;
    });
}

void dfsph_apply_kappas_to_velocities_from_solids(
  size_t const particle_count,
  float const dt,
  float const target_density,
  V2f* const velocities,
  float const* const kappas,
  float const* const densities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<uint8_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = solid_neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const kappa_over_density =
          safe_divide(kappas[particle_index], densities[particle_index])
            .value_or(0.0f);
        V2f neg_acceleration_sum{0.0f, 0.0f};

        auto const& nbhd_indices = solid_neighbor_indices[particle_index];
        auto const& nbhd_kernel_gradients =
          solid_neighbor_kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const other_density_times_volume =
              solid_volumes[other_particle_index] * target_density;

            neg_acceleration_sum += other_density_times_volume *
                                    kappa_over_density *
                                    nbhd_kernel_gradients[j];
        }

        velocities[particle_index] -= dt * neg_acceleration_sum;
    });
}

}  // namespace emerald::sph2d_box
