#include <emerald/sph2d_box/pcisph_ops.h>

#include <emerald/sph_common/common.h>
#include <emerald/sph_common/kernels.h>
#include <emerald/util/functions.h>
#include <emerald/util/random.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace emerald::sph2d_box {

using namespace emerald::util;

void pcisph_compute_densities(
  size_t const particle_count,
  float const mass_per_particle,
  float const support,
  float* const densities,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_kernels) {
    auto const w_0 = kernels::W(0.0f, support);
    for_each_iota(particle_count, [=](auto const i) {
        auto const nbhd_count = neighbor_counts[i];
        if (!nbhd_count) {
            densities[i] = mass_per_particle * w_0;
            return;
        }

        float w_sum = w_0;
        auto const& nbhd_kernels = neighbor_kernels[i];
        for (uint8_t j = 0; j < nbhd_count; ++j) { w_sum += nbhd_kernels[j]; }
        densities[i] = mass_per_particle * w_sum;
    });
}

void pcisph_accumulate_density_from_solids(
  size_t const particle_count,
  float const target_density,
  float* const densities,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<float> const* const solid_neighbor_kernels) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = solid_neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        float volume_fraction = 0.0f;
        auto const& nbhd_indices = solid_neighbor_indices[particle_index];
        auto const& nbhd_kernels = solid_neighbor_kernels[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            volume_fraction +=
              solid_volumes[other_particle_index] * nbhd_kernels[j];
        }

        densities[particle_index] += target_density * volume_fraction;
    });
}

void pcisph_update_pressures(size_t const particle_count,
                             float const target_density,
                             float const pressure_correction_denom,
                             float* const pressures,
                             float const* const densities) {
    for_each_iota(particle_count, [=](auto const i) {
        auto const density_error =
          std::max(densities[i] - target_density, 0.0f);
        auto const pressure_tilda = density_error / pressure_correction_denom;
        pressures[i] += pressure_tilda;
    });
}

void pcisph_accumulate_pressure_forces(
  size_t const particle_count,
  float const mass_per_particle,
  float const support,
  float const viscosity,
  V2f* const pressure_forces,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
  Neighbor_values<float> const* const neighbor_distances,
  Neighbor_values<V2f> const* const neighbor_vectors_to,
  Neighbor_values<V2f> const* const neighbor_kernel_gradients,
  V2f const* const velocities,
  float const* const pressures,
  float const* const densities) {
    auto const H = support;
    auto const M = mass_per_particle;
    auto const N2 = 0.01f * H * H;
    constexpr float alpha = 1.0f;
    constexpr float beta = 2.0f;

    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const velocity = velocities[particle_index];
        auto const pressure = pressures[particle_index];
        auto const density = densities[particle_index];
        auto const& nbhd_indices = neighbor_indices[particle_index];
        auto const& nbhd_distances = neighbor_distances[particle_index];
        auto const& nbhd_vectors_to = neighbor_vectors_to[particle_index];
        auto const& nbhd_grads_w = neighbor_kernel_gradients[particle_index];
        auto const p_over_rho_sqr = pressure / sqr(density);
        V2f pressure_force{0.0f, 0.0f};
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const vector_to_other = nbhd_vectors_to[j];
            auto const grad_w = nbhd_grads_w[j];

            auto const other_velocity = velocities[other_particle_index];
            auto const other_pressure = pressures[other_particle_index];
            auto const other_density = densities[other_particle_index];

            pressure_force +=
              (p_over_rho_sqr + (other_pressure / sqr(other_density))) * grad_w;

            // Add viscosity.
            // This is a very non-linear force, so it is done in
            // the predictor
            auto const delta_vel = other_velocity - velocity;
            auto const dot_vab_rab = vector_to_other.dot(delta_vel);
            if (dot_vab_rab < 0) {
                auto const distance = nbhd_distances[j];
                auto const mu_ab = H * dot_vab_rab / (distance + N2);
                auto const den_avg = (density + other_density) / 2;

                pressure_force +=
                  (((-alpha * viscosity * mu_ab) + (beta * sqr(mu_ab))) /
                   den_avg) *
                  grad_w;
            }
        }

        pressure_forces[particle_index] += -(M * M) * pressure_force;
    });
}

void pcisph_accumulate_pressure_forces_from_solids(
  size_t const particle_count,
  float const mass_per_particle,
  float const target_density,
  V2f* const pressure_forces,
  float const* const solid_volumes,
  uint8_t const* const solid_neighbor_counts,
  Neighbor_values<size_t> const* const solid_neighbor_indices,
  Neighbor_values<V2f> const* const solid_neighbor_kernel_gradients,
  float const* const pressures,
  float const* const densities) {
    auto const M = mass_per_particle;

    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = solid_neighbor_counts[particle_index];
        if (!nbhd_count) { return; }

        auto const pressure = pressures[particle_index];
        auto const density = densities[particle_index];
        auto const& nbhd_indices = solid_neighbor_indices[particle_index];
        auto const& nbhd_grads_w =
          solid_neighbor_kernel_gradients[particle_index];
        auto const p_over_rho_sqr = pressure / sqr(density);
        V2f volume_gradient_sum{0.0f, 0.0f};
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const other_particle_index = nbhd_indices[j];
            auto const grad_w = nbhd_grads_w[j];
            auto const other_volume = solid_volumes[other_particle_index];
            volume_gradient_sum += other_volume * grad_w;
        }

        pressure_forces[particle_index] +=
          (-M * target_density * p_over_rho_sqr) * volume_gradient_sum;
    });
}

}  // namespace emerald::sph2d_box
