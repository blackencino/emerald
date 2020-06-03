
//------------------------------------------------------------------------------
void dfsph_p_compute_shared_coeffs(
  size_t const particle_count,
  float const dt,
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
template <typename T>
static inline void update_max(std::atomic<T>& atom, T const val) {
    for (T atom_val = atom;
         atom_val < val && !atom.compare_exchange_weak(
                             atom_val, val, std::memory_order_relaxed);) {}
}

static std::pair<float, float> dfsph_p_compute_density_stars_partial(
  size_t const particle_count,
  bool const boundary,
  float const dt,
  float const target_density,
  float* const density_stars,
  float const* const densities,
  V2f const* const self_velocities,

  float const* const neighbor_volumes,
  V2f const* const neighbor_velocities,
  Neighborhood_pointers const neighborhood) {
    if (!particle_count) { return {0.0f, 0.0f}; }

    // Also computes the average density variation
    std::atomic<uint64_t> integral_error_sum = 0;
    std::atomic<float> error_max = 0.0f;

    for_each_iota(
      particle_count,
      [=, &integral_error_sum, &error_max](auto const particle_index) {
          auto const nbhd_count = neighborhood.counts[particle_index];
          if (!nbhd_count) {
              if (!boundary) {
                  density_stars[particle_index] = densities[particle_index];
              } else {
                  auto const density_star =
                    (density_stars[particle_index] =
                       std::max(0.0f, density_stars[particle_index]));
                  auto const error = std::max(
                    0.0f,
                    safe_divide(density_star - target_density, target_density)
                      .value_or(0.0f));
                  update_max(error_max, error);
                  integral_error_sum += static_cast<uint64_t>(error * 1000.0f);
              }
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
                target_density * fluid_volumes[neighbor_particle_index];
              auto const grad_w = nbhd_kernel_gradients[j];
              auto const neighbor_velocity =
                neighbor_velocities[neighbor_particle_index];

              divergence += neighbor_mass *
                            ((self_velocity - neighbor_velocity).dot(grad_w));
          }

          if (!boundary) {
              density_stars[particle_index] =
                densities[particle_index] + dt * divergence;
          } else {
              auto const density_star =
                (density_stars[particle_index] += dt * divergence);
              auto const error = std::max(
                0.0f,
                safe_divide(density_star - target_density, target_density)
                  .value_or(0.0f));
              update_max(error_max, error);
              integral_error_sum += static_cast<uint64_t>(error * 1000.0f);
          }
      });

    auto const error_average_numer = static_cast<double>(integral_error_sum);
    auto const error_average_denom = static_cast<double>(1000 * particle_count);
    return {error_average_numer / error_average_denom, error_max};
}

std::pair<float, float> dfsph_p_compute_density_stars(
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
    dfsph_p_compute_density_stars_partial(particle_count,
                                          false,
                                          dt,
                                          target_density,
                                          density_stars,
                                          densities,
                                          fluid_velocities,
                                          fluid_volumes,
                                          fluid_velocities,
                                          fluid_neighborhood);
    return dfsph_p_compute_density_stars_partial(particle_count,
                                                 true,
                                                 dt,
                                                 target_density,
                                                 density_stars,
                                                 nullptr,
                                                 fluid_velocities,
                                                 solid_volumes,
                                                 solid_velocities,
                                                 solid_neighborhood);
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
              target_density * fluid_volumes[neighbor_particle_index];
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

  float const* const fluid_neighbor_volumes,
  V2f const* const fluid_neighbor_velocities,
  Neighborhood_pointers const fluid_neighborhood,

  float const* const solid_neighbor_volumes,
  V2f const* const solid_neighbor_velocities,
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

// CJH TODO MAKE DIVERGENCE CALC RETURN ERRORS

//------------------------------------------------------------------------------
void dfsph_p_compute_divergence_pseudo_pressures(
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