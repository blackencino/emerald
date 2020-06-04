#include <emerald/sph2d_box/iisph_ap_ops.h>

#include <emerald/sph_common/common.h>
#include <emerald/sph_common/kernels.h>
#include <emerald/util/format.h>

#include <fmt/format.h>

#include <algorithm>
#include <atomic>
#include <utility>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;
using namespace emerald::util;

//------------------------------------------------------------------------------
void iisph_ap_density_stars_and_diagonals(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const density_stars,
  float* const diagonals,

  float const* const densities,

  float const* const fluid_volumes,
  V2f const* const fluid_velocities,
  Neighborhood_pointers const fluid_neighborhood,

  float const* const solid_volumes,
  V2f const* const solid_velocities,
  Neighborhood_pointers const solid_neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const fluid_nbhd_count = fluid_neighborhood.counts[particle_index];
        auto const solid_nbhd_count = solid_neighborhood.counts[particle_index];
        if (!(fluid_nbhd_count + solid_nbhd_count)) {
            density_stars[particle_index] = densities[particle_index];
            diagonals[particle_index] = 0.0f;
            return;
        }

        auto const self_mass = fluid_volumes[particle_index] * target_density;
        auto const self_density = densities[particle_index];
        auto const self_velocity = fluid_velocities[particle_index];

        V2f sum_mf_gradwif{0.0f, 0.0f};
        float sum_mf_gradwif_dot_gradwif = 0.0f;
        V2f sum_mb_gradwib{0.0f, 0.0f};

        float divergence = 0.0f;

        // Loop over fluids
        if (fluid_nbhd_count) {
            auto const& nbhd_indices =
              fluid_neighborhood.indices[particle_index];
            auto const& nbhd_kernel_gradients =
              fluid_neighborhood.kernel_gradients[particle_index];
            for (uint8_t j = 0; j < fluid_nbhd_count; ++j) {
                auto const neighbor_particle_index = nbhd_indices[j];
                auto const grad_w = nbhd_kernel_gradients[j];
                auto const neighbor_mass =
                  fluid_volumes[neighbor_particle_index] * target_density;

                sum_mf_gradwif += neighbor_mass * grad_w;
                sum_mf_gradwif_dot_gradwif +=
                  neighbor_mass * grad_w.dot(grad_w);

                auto const delta_vel =
                  self_velocity - fluid_velocities[neighbor_particle_index];
                divergence += neighbor_mass * delta_vel.dot(grad_w);
            }
        }

        // Loop over solids
        if (solid_nbhd_count) {
            auto const& nbhd_indices =
              solid_neighborhood.indices[particle_index];
            auto const& nbhd_kernel_gradients =
              solid_neighborhood.kernel_gradients[particle_index];
            for (uint8_t j = 0; j < solid_nbhd_count; ++j) {
                auto const neighbor_particle_index = nbhd_indices[j];
                auto const grad_w = nbhd_kernel_gradients[j];
                auto const neighbor_mass =
                  solid_volumes[neighbor_particle_index] * target_density;

                sum_mb_gradwib += neighbor_mass * grad_w;

                auto const delta_vel =
                  self_velocity - solid_velocities[neighbor_particle_index];
                divergence += neighbor_mass * delta_vel.dot(grad_w);
            }
        }

        auto const self_denom = sqr(self_density);
        auto const target_denom = sqr(target_density);

        if (safe_divide(sum_mf_gradwif, self_denom) &&
            safe_divide(sum_mb_gradwib, self_denom) &&
            safe_divide(sum_mb_gradwib, target_denom) &&
            safe_divide(sum_mf_gradwif_dot_gradwif, self_denom)) {
            auto const aii =
              (sum_mf_gradwif + sum_mb_gradwib)
                .dot(sum_mf_gradwif / sqr(self_density) +
                     sum_mb_gradwib / sqr(self_density)) +
              self_mass * sum_mf_gradwif_dot_gradwif / sqr(self_density);

            diagonals[particle_index] = -(sqr(dt) * aii);
        } else {
            diagonals[particle_index] = 0.0f;
        }

        density_stars[particle_index] = self_density + dt * divergence;
    });
}

std::pair<float, float> iisph_ap_iterate_pressures_in_place(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float const omega,
  float* const pressures,
  float const* const density_stars,
  V2f const* const pressure_accelerations,
  float const* const diagonals,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood) {
    if (!particle_count) { return {0.0f, 0.0f}; }

    // Also computes the average density variation
    std::atomic<uint64_t> integral_error_sum = 0;
    std::atomic<float> error_max = 0.0f;

    for_each_iota(
      particle_count,
      [=, &integral_error_sum, &error_max](auto const particle_index) {
          auto const fluid_nbhd_count =
            fluid_neighborhood.counts[particle_index];
          auto const solid_nbhd_count =
            solid_neighborhood.counts[particle_index];
          if (!fluid_nbhd_count && !solid_nbhd_count) {
              pressures[particle_index] = 0.0f;
              return;
          }

          float Api = 0.0f;
          auto const self_pressure_acceleration =
            pressure_accelerations[particle_index];

          if (fluid_nbhd_count) {
              auto const& nbhd_indices =
                fluid_neighborhood.indices[particle_index];
              auto const& nbhd_kernel_gradients =
                fluid_neighborhood.kernel_gradients[particle_index];
              for (uint8_t j = 0; j < fluid_nbhd_count; ++j) {
                  auto const neighbor_particle_index = nbhd_indices[j];
                  auto const neighbor_mass =
                    target_density * fluid_volumes[neighbor_particle_index];
                  auto const grad_w = nbhd_kernel_gradients[j];
                  auto const neighbor_pressure_acceleration =
                    pressure_accelerations[neighbor_particle_index];

                  Api += neighbor_mass * ((self_pressure_acceleration -
                                           neighbor_pressure_acceleration)
                                            .dot(grad_w));
              }
          }

          if (solid_nbhd_count) {
              auto const& nbhd_indices =
                solid_neighborhood.indices[particle_index];
              auto const& nbhd_kernel_gradients =
                solid_neighborhood.kernel_gradients[particle_index];
              for (uint8_t j = 0; j < solid_nbhd_count; ++j) {
                  auto const neighbor_particle_index = nbhd_indices[j];
                  auto const neighbor_mass =
                    target_density * solid_volumes[neighbor_particle_index];
                  auto const grad_w = nbhd_kernel_gradients[j];

                  Api +=
                    neighbor_mass * (self_pressure_acceleration.dot(grad_w));
              }
          }

          Api *= sqr(dt);

          auto const source = target_density - density_stars[particle_index];
          auto const diagonal = diagonals[particle_index];
          auto const error = std::max(
            0.0f, safe_divide(Api - source, target_density).value_or(0.0f));
          // fmt::print("Error {}: {}\n", particle_index, error);
          update_atomic_max(error_max, error);
          integral_error_sum += static_cast<uint64_t>(error * 1000.0f);

          auto const old_pressure = pressures[particle_index];
          auto const numer = omega * (source - Api);
          pressures[particle_index] = std::max(
            0.0f, old_pressure + safe_divide(numer, diagonal).value_or(0.0f));
      });

    auto const error_average_numer = static_cast<double>(integral_error_sum);
    auto const error_average_denom = static_cast<double>(1000 * particle_count);
    return {error_average_numer / error_average_denom, error_max};
}

}  // namespace emerald::sph2d_box
