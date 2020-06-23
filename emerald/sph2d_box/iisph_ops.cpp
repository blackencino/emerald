#include <emerald/sph2d_box/iisph_ops.h>

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
// Compute Diis
static void iisph_compute_diis_partial(
  size_t const particle_count,
  bool const boundary,
  float const dt,
  float const target_density,
  V2f* const diis,
  float const* const self_densities,
  float const* const neighbor_volumes,
  Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            if (!boundary) { diis[particle_index] = V2f{0.0f, 0.0f}; }
            return;
        }

        auto const density = self_densities[particle_index];

        V2f m_grad_sum{0.0f, 0.0f};
        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            float const mass =
              target_density * neighbor_volumes[neighbor_particle_index];
            m_grad_sum += mass * nbhd_kernel_gradients[j];
        }

        auto const numer = -sqr(dt) * m_grad_sum;
        auto const denom = sqr(density);
        auto const dii = safe_divide(numer, denom).value_or(V2f{0.0f, 0.0f});

        if (boundary) {
            diis[particle_index] += dii;
        } else {
            diis[particle_index] = dii;
        }
    });
}

void iisph_compute_diis(size_t const particle_count,
                        float const dt,
                        float const target_density,
                        V2f* const diis,

                        float const* const self_densities,

                        float const* const fluid_volumes,
                        Neighborhood_pointers const fluid_neighborhood,

                        float const* const solid_volumes,
                        Neighborhood_pointers const solid_neighborhood) {
    iisph_compute_diis_partial(particle_count,
                               false,
                               dt,
                               target_density,
                               diis,
                               self_densities,
                               fluid_volumes,
                               fluid_neighborhood);

    iisph_compute_diis_partial(particle_count,
                               true,
                               dt,
                               target_density,
                               diis,
                               self_densities,
                               solid_volumes,
                               solid_neighborhood);
}

//------------------------------------------------------------------------------
// Aiis and Density Stars
static void iisph_compute_aiis_and_density_stars_partial(
  size_t const particle_count,
  bool const boundary,
  float const dt,
  float const target_density,
  float* const aiis,
  float* const density_stars,

  V2f const* const diis,
  float const* const densities,

  float const* const self_volumes,
  V2f const* const self_velocities,

  float const* const neighbor_volumes,
  V2f const* const neighbor_velocities,
  Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            if (!boundary) {
                density_stars[particle_index] = densities[particle_index];
                aiis[particle_index] = 0.0f;
            }
            return;
        }

        auto const dii = diis[particle_index];
        auto const self_velocity = self_velocities[particle_index];

        float aii = 0.0f;
        float delta_density = 0.0f;
        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            auto const neighbor_volume =
              neighbor_volumes[neighbor_particle_index];
            auto const neighbor_velocity =
              neighbor_velocities[neighbor_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];
            auto const mass = neighbor_volume * target_density;

            if (boundary) {
                aii += mass * (dii.dot(grad_w));
            } else {
                auto const neighbor_density =
                  densities[neighbor_particle_index];
                auto const numer = -sqr(dt) * mass * grad_w;
                auto const denom = sqr(neighbor_density);
                if (is_safe_divide(numer, denom)) {
                    auto const dij = numer / denom;
                    auto const dji = -dij;
                    aii += mass * ((dii - dji).dot(grad_w));
                }
            }

            // fmt::print("delta_vel: {}, grad_w: {}, dot: {}\n",
            //            (self_velocity - neighbor_velocity),
            //            grad_w,
            //            (self_velocity - neighbor_velocity).dot(grad_w));

            delta_density +=
              dt * mass * ((self_velocity - neighbor_velocity).dot(grad_w));
        }

        if (boundary) {
            // fmt::print("{}: boundary delta density: {}\n",
            //            particle_index,
            //            delta_density);
            aiis[particle_index] += aii;
            density_stars[particle_index] += delta_density;
        } else {
            // fmt::print(
            //   "{}: fluid delta density: {}\n", particle_index,
            //   delta_density);
            aiis[particle_index] = aii;
            density_stars[particle_index] =
              densities[particle_index] + delta_density;
        }
    });
}

void iisph_compute_aiis_and_density_stars(
  size_t const particle_count,
  float const dt,
  float const target_density,
  float* const aiis,
  float* const density_stars,

  V2f const* const diis,
  float const* const densities,

  float const* const fluid_volumes,
  V2f const* const fluid_velocities,
  Neighborhood_pointers const fluid_neighborhood,

  float const* const solid_volumes,
  V2f const* const solid_velocities,
  Neighborhood_pointers const solid_neighborhood) {
    iisph_compute_aiis_and_density_stars_partial(particle_count,
                                                 false,
                                                 dt,
                                                 target_density,
                                                 aiis,
                                                 density_stars,
                                                 diis,
                                                 densities,
                                                 fluid_volumes,
                                                 fluid_velocities,
                                                 fluid_volumes,
                                                 fluid_velocities,
                                                 fluid_neighborhood);
    iisph_compute_aiis_and_density_stars_partial(particle_count,
                                                 true,
                                                 dt,
                                                 target_density,
                                                 aiis,
                                                 density_stars,
                                                 diis,
                                                 nullptr,
                                                 fluid_volumes,
                                                 fluid_velocities,
                                                 solid_volumes,
                                                 solid_velocities,
                                                 solid_neighborhood);
}

//------------------------------------------------------------------------------
// These are zero for solids, so only need to iterate over fluids.
void iisph_compute_sum_dij_pjs(size_t const particle_count,
                               float const dt,
                               float const target_density,
                               V2f* const sum_dij_pjs,
                               float const* const volumes,
                               float const* const densities,
                               float const* const pressures,
                               Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            sum_dij_pjs[particle_index] = V2f{0.0f, 0.0f};
            return;
        }

        V2f sum_dij_pj{0.0f, 0.0f};
        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            auto const mass = volumes[neighbor_particle_index] * target_density;
            auto const density = densities[neighbor_particle_index];
            auto const pressure = pressures[neighbor_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];

            auto const numer = -sqr(dt) * mass * pressure * grad_w;
            auto const denom = sqr(density);
            if (is_safe_divide(numer, denom)) { sum_dij_pj += numer / denom; }
        }

        sum_dij_pjs[particle_index] = sum_dij_pj;
    });
}

//------------------------------------------------------------------------------
static void iisph_compute_betas_partial(
  size_t const particle_count,
  bool const boundary,
  float dt,
  float const target_density,
  float* const betas,
  float const* const densities,
  float const* const pressures,
  V2f const* const diis,
  V2f const* const sum_dij_pjs,
  float const* neighbor_volumes,
  Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            if (!boundary) { betas[particle_index] = 0.0f; }
            return;
        }

        auto const pressure_i = pressures[particle_index];

        float beta = 0.0f;
        auto const sum_dij_pj = sum_dij_pjs[particle_index];
        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            auto const grad_w = nbhd_kernel_gradients[j];
            auto const mass =
              target_density * neighbor_volumes[neighbor_particle_index];
            if (boundary) {
                beta += mass * (sum_dij_pj.dot(grad_w));
            } else {
                auto const density_j = densities[neighbor_particle_index];
                auto const pressure_j = pressures[neighbor_particle_index];
                auto const djj = diis[neighbor_particle_index];
                auto const sum_djk_pk = sum_dij_pjs[neighbor_particle_index];
                auto const dij =
                  safe_divide(-sqr(dt) * mass * grad_w, sqr(density_j))
                    .value_or(V2f{0.0f, 0.0f});
                auto const dji = -dij;
                auto const sum_djk_pk_k_not_i = sum_djk_pk - (dji * pressure_i);

                beta +=
                  mass * ((sum_dij_pj - (djj * pressure_j) - sum_djk_pk_k_not_i)
                            .dot(grad_w));
            }
        }

        if (boundary) {
            betas[particle_index] += beta;
        } else {
            betas[particle_index] = beta;
        }
    });
}

void iisph_compute_betas(size_t const particle_count,
                         float dt,
                         float const target_density,
                         float* const betas,
                         float const* const densities,
                         float const* const pressures,
                         V2f const* const diis,
                         V2f const* const sum_dij_pjs,
                         float const* const fluid_volumes,
                         Neighborhood_pointers const fluid_neighborhood,
                         float const* const solid_volumes,
                         Neighborhood_pointers const solid_neighborhood) {
    iisph_compute_betas_partial(particle_count,
                                false,
                                dt,
                                target_density,
                                betas,
                                densities,
                                pressures,
                                diis,
                                sum_dij_pjs,
                                fluid_volumes,
                                fluid_neighborhood);
    iisph_compute_betas_partial(particle_count,
                                true,
                                dt,
                                target_density,
                                betas,
                                nullptr,
                                pressures,
                                diis,
                                sum_dij_pjs,
                                solid_volumes,
                                solid_neighborhood);
}

//------------------------------------------------------------------------------
// Returns error average and max
std::pair<float, float> iisph_compute_new_pressures(
  size_t const particle_count,
  float const omega,
  float const target_density,
  float* const betas_in_new_pressures_out,
  float const* const old_pressures,
  float const* const alphas,
  float const* const density_stars) {
    if (!particle_count) { return {0.0f, 0.0f}; }

    // Also computes the average density variation
    std::atomic<uint64_t> integral_error_sum = 0;
    std::atomic<float> error_max = 0.0f;

    for_each_iota(
      particle_count,
      [=, &integral_error_sum, &error_max](auto const particle_index) {
          auto const alpha = alphas[particle_index];
          auto const old_pressure = old_pressures[particle_index];
          if (alpha == 0.0f) {
              betas_in_new_pressures_out[particle_index] = old_pressure;
              return;
          }
          auto const density_star = density_stars[particle_index];
          // fmt::print("Density_star: {}\n", density_star);
          auto const beta = betas_in_new_pressures_out[particle_index];

          auto const numer = target_density - density_star - beta;
          if (!is_safe_divide(numer, alpha)) {
              betas_in_new_pressures_out[particle_index] = old_pressure;
              fmt::print(
                "ERROR: safe_divide fail in compute new pressures. numer: {}, "
                "denom: {}, nbhd: {}, {}\n",
                numer,
                alpha);
              return;
          }

          // si - Api + aiipi
          // si - (Api - aiipi)
          // si - beta
          // beta = Api - aiipi
          // Api = beta + aiipi
          //
          // error = si - Api
          // numer = si - beta
          // numer = si - (Api - aiipi)
          // numer = si - Api + aiipi
          // numer = error + aiipi
          // numer - aiipi = error
          auto const error =
            std::max(0.0f, -(numer - (alpha * old_pressure)) / target_density);
          // fmt::print("Error {}: {}\n", particle_index, error);
          update_atomic_max(error_max, error);
          integral_error_sum += static_cast<uint64_t>(error * 1000.0f);

          auto const new_pressure = numer / alpha;
          // fmt::print("Pressure {}: old: {}, new: {}\n",
          //            particle_index,
          //            old_pressure,
          //            new_pressure);
          betas_in_new_pressures_out[particle_index] =
            std::max(0.0f, mix(old_pressure, new_pressure, omega));
      });

    auto const error_average_numer = static_cast<double>(integral_error_sum);
    auto const error_average_denom = static_cast<double>(1000 * particle_count);
    return {error_average_numer / error_average_denom, error_max};
}

}  // namespace emerald::sph2d_box