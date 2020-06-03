#include <emerald/sph2d_box/dfsph_p_ops.h>

#include <emerald/sph2d_box/iisph_ap_ops.h>
#include <emerald/sph2d_box/iisph_pseudo_ap_ops.h>
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
void dfsph_p_compute_shared_coeffs(
  size_t const particle_count,
  float const target_density,
  float* const shared_coeffs,
  float const* const densities,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const fluid_nbhd_count = fluid_neighborhood.counts[particle_index];
        auto const solid_nbhd_count = solid_neighborhood.counts[particle_index];
        if (!(fluid_nbhd_count + solid_nbhd_count)) {
            shared_coeffs[particle_index] = 0.0f;
            return;
        }

        auto const self_density = densities[particle_index];

        V2f sum_mgradw{0.0f, 0.0f};
        float sum_mgradw_dot_mgradw = 0.0f;

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

                auto const mgradw = neighbor_mass * grad_w;

                sum_mgradw += mgradw;
                sum_mgradw_dot_mgradw += mgradw.dot(mgradw);
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

                auto const mgradw = neighbor_mass * grad_w;

                sum_mgradw += mgradw;
            }
        }

        auto const numer = sqr(self_density);
        auto const denom = sum_mgradw.dot(sum_mgradw) + sum_mgradw_dot_mgradw;

        shared_coeffs[particle_index] =
          safe_divide(numer, denom).value_or(0.0f);
    });
}

//------------------------------------------------------------------------------
static void dfsph_p_compute_divergences_partial(
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

void dfsph_p_compute_divergences(
  size_t const particle_count,
  float const target_density,
  float* const divergences,
  float const* const fluid_volumes,
  V2f const* const fluid_velocities,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  V2f const* const solid_velocities,
  Neighborhood_pointers const solid_neighborhood) {
    dfsph_p_compute_divergences_partial(particle_count,
                                        false,
                                        target_density,
                                        divergences,
                                        fluid_velocities,
                                        fluid_volumes,
                                        fluid_velocities,
                                        fluid_neighborhood);

    dfsph_p_compute_divergences_partial(particle_count,
                                        true,
                                        target_density,
                                        divergences,
                                        fluid_velocities,
                                        solid_volumes,
                                        solid_velocities,
                                        solid_neighborhood);
}

//------------------------------------------------------------------------------
void dfsph_p_compute_density_stars(
  size_t const particle_count,
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
    dfsph_p_compute_divergences(particle_count,
                                target_density,
                                density_stars,
                                fluid_volumes,
                                fluid_velocities,
                                fluid_neighborhood,
                                solid_volumes,
                                solid_velocities,
                                solid_neighborhood);

    for_each_iota(particle_count, [=](auto const particle_index) {
        auto& d = density_stars[particle_index];
        d = std::max(0.0f, densities[particle_index] + (dt * d));
    });
}

//------------------------------------------------------------------------------
void dfsph_p_compute_density_pseudo_pressures(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const density_pseudo_pressures,
  float const* const density_stars,
  float const* const shared_coeffs) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const delta_density =
          density_stars[particle_index] - target_density;
        if (delta_density <= 0.0f) {
            density_pseudo_pressures[particle_index] = 0.0f;
        } else {
            density_pseudo_pressures[particle_index] =
              safe_divide(delta_density * shared_coeffs[particle_index],
                          sqr(dt))
                .value_or(0.0f);
        }
    });
}

//------------------------------------------------------------------------------
void dfsph_p_compute_divergence_pseudo_pressures(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const divergence_pseudo_pressures,
  float const* const divergences,
  float const* const shared_coeffs) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const divergence = divergences[particle_index];
        if (divergence <= 0.0f) {
            divergence_pseudo_pressures[particle_index] = 0.0f;
        } else {
            divergence_pseudo_pressures[particle_index] =
              safe_divide(divergence * shared_coeffs[particle_index], dt)
                .value_or(0.0f);
        }
    });
}

//------------------------------------------------------------------------------
void dfsph_p_integrate_pseudo_pressures(
  size_t const particle_count,
  float const dt,
  float const target_density,
  V2f* const velocity_stars,
  V2f* const pressure_accelerations,
  float const* const pseudo_pressures,
  float const* const densities,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood) {
    iisph_ap_compute_pressure_accelerations(particle_count,
                                            target_density,
                                            pressure_accelerations,
                                            pseudo_pressures,
                                            densities,
                                            fluid_volumes,
                                            fluid_neighborhood,
                                            solid_volumes,
                                            solid_neighborhood);

    accumulate(particle_count, dt, velocity_stars, pressure_accelerations);
}

}  // namespace emerald::sph2d_box
