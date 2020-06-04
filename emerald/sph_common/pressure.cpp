#include <emerald/sph_common/pressure.h>

#include <emerald/sph_common/common.h>
#include <emerald/util/functions.h>
#include <emerald/util/safe_divide.h>

namespace emerald::sph_common {

static void apply_pressures_in_place_partial(
  size_t const particle_count,
  bool const boundary,
  float const dt,
  float const target_density,
  V2f* const velocities,
  float const* const densities,
  float const* const pressures,
  float const* const neighbor_volumes,
  Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) { return; }

        auto const p_over_rho_sqr =
          safe_divide(pressures[particle_index], sqr(densities[particle_index]))
            .value_or(0.0f);

        V2f acceleration{0.0f, 0.0f};
        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            auto const neighbor_particle_index = nbhd_indices[j];
            auto const mass =
              target_density * neighbor_volumes[neighbor_particle_index];
            auto const grad_w = nbhd_kernel_gradients[j];
            if (boundary) {
                acceleration += -mass * p_over_rho_sqr * grad_w;
            } else {
                auto const nbr_p_over_rho_sqr =
                  safe_divide(pressures[neighbor_particle_index],
                              sqr(densities[neighbor_particle_index]))
                    .value_or(0.0f);
                acceleration +=
                  -mass * (p_over_rho_sqr + nbr_p_over_rho_sqr) * grad_w;
            }
        }

        velocities[particle_index] += dt * acceleration;
    });
}

void apply_pressures_in_place(size_t const particle_count,
                              float const dt,
                              float const target_density,
                              V2f* const velocities,
                              float const* const densities,
                              float const* const pressures,
                              float const* const fluid_volumes,
                              Neighborhood_pointers const fluid_neighborhood,
                              float const* const solid_volumes,
                              Neighborhood_pointers const solid_neighborhood) {
    apply_pressures_in_place_partial(particle_count,
                                     false,
                                     dt,
                                     target_density,
                                     velocities,
                                     densities,
                                     pressures,
                                     fluid_volumes,
                                     fluid_neighborhood);
    apply_pressures_in_place_partial(particle_count,
                                     true,
                                     dt,
                                     target_density,
                                     velocities,
                                     densities,
                                     pressures,
                                     solid_volumes,
                                     solid_neighborhood);
}

//------------------------------------------------------------------------------
static void compute_pressure_accelerations_partial(
  size_t const particle_count,
  bool const boundary,
  float const target_density,

  V2f* const pressure_accelerations,

  float const* const pressures,
  float const* const densities,

  float const* const neighbor_volumes,
  Neighborhood_pointers const neighborhood) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighborhood.counts[particle_index];
        if (!nbhd_count) {
            if (!boundary) {
                pressure_accelerations[particle_index] = {0.0f, 0.0f};
            }
            return;
        }

        auto const self_pressure = pressures[particle_index];
        auto const self_density = densities[particle_index];
        auto const p_over_rho_sqr =
          safe_divide(self_pressure, sqr(self_density)).value_or(0.0f);

        V2f pressure_acceleration{0.0f, 0.0f};
        auto const& nbhd_indices = neighborhood.indices[particle_index];
        auto const& nbhd_kernel_gradients =
          neighborhood.kernel_gradients[particle_index];
        if (boundary) {
            auto const coeff = p_over_rho_sqr;
            for (uint8_t j = 0; j < nbhd_count; ++j) {
                auto const neighbor_particle_index = nbhd_indices[j];
                auto const neighbor_mass =
                  target_density * neighbor_volumes[neighbor_particle_index];
                auto const grad_w = nbhd_kernel_gradients[j];

                pressure_acceleration += (-neighbor_mass * coeff) * grad_w;
            }
        } else {
            for (uint8_t j = 0; j < nbhd_count; ++j) {
                auto const neighbor_particle_index = nbhd_indices[j];
                auto const neighbor_mass =
                  target_density * neighbor_volumes[neighbor_particle_index];
                auto const grad_w = nbhd_kernel_gradients[j];

                auto const neighbor_pressure =
                  pressures[neighbor_particle_index];
                auto const neighbor_density =
                  densities[neighbor_particle_index];
                auto const coeff =
                  p_over_rho_sqr +
                  safe_divide(neighbor_pressure, sqr(neighbor_density))
                    .value_or(0.0f);

                pressure_acceleration += (-neighbor_mass * coeff) * grad_w;
            }
        }

        if (boundary) {
            pressure_accelerations[particle_index] += pressure_acceleration;
        } else {
            pressure_accelerations[particle_index] = pressure_acceleration;
        }
    });
}

void compute_pressure_accelerations(
  size_t const particle_count,
  float const target_density,
  V2f* const pressure_accelerations,
  float const* const pressures,
  float const* const densities,
  float const* const fluid_volumes,
  Neighborhood_pointers const fluid_neighborhood,
  float const* const solid_volumes,
  Neighborhood_pointers const solid_neighborhood) {
    compute_pressure_accelerations_partial(particle_count,
                                           false,
                                           target_density,
                                           pressure_accelerations,
                                           pressures,
                                           densities,
                                           fluid_volumes,
                                           fluid_neighborhood);

    compute_pressure_accelerations_partial(particle_count,
                                           true,
                                           target_density,
                                           pressure_accelerations,
                                           pressures,
                                           densities,
                                           solid_volumes,
                                           solid_neighborhood);
}

}  // namespace emerald::sph_common
