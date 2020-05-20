#include <emerald/sph2d_box/sim_ops.h>

#include <emerald/sph2d_box/kernels.h>
#include <emerald/util/functions.h>
#include <emerald/util/random.h>
#include <emerald/z_index/z_index.h>

#include <fmt/format.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_scan.h>
#include <tbb/parallel_sort.h>

#include <algorithm>
#include <cmath>

namespace emerald::sph2d_box {

using namespace emerald::util;

void accumulate_gravity_forces(size_t const particle_count,
                               float const mass_per_particle,
                               float const gravity,
                               V2f* const forces) {
    for_each_iota(particle_count, [=](auto const i) {
        forces[i][1] -= mass_per_particle * gravity;
    });
}

void accumulate_constant_pole_attraction_forces(size_t const particle_count,
                                                float const magnitude,
                                                V2f const pole,
                                                V2f* const forces,
                                                V2f const* const positions) {
    for_each_iota(particle_count, [=](auto const i) {
        forces[i] += magnitude * ((pole - positions[i]).normalized());
    });
}

// This isn't the best drag force, it needs to be conscious of timestep.
void accumulate_simple_drag_forces(size_t const particle_count,
                                   float const magnitude,
                                   float const particle_diameter,
                                   V2f* const forces,
                                   V2f const* const velocities) {
    accumulate(particle_count, -magnitude * particle_diameter, velocities);
}

void accumulate_anti_coupling_repulsive_forces(
  size_t const particle_count,
  float const max_distance,
  float const force_magnitude,
  V2f* const forces,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_distances) {
    std::uniform_real_distribution<float> angle_dist{float(-M_PI), float(M_PI)};
    for_each_iota(particle_count, [=](auto const i) {
        auto const nbhd_count = neighbor_counts[i];
        if (!nbhd_count) { return; }

        auto const& distances = neighbor_distances[i];

        for (uint8_t j = 0; j < nbhd_count; ++j) {
            if (distances[j] < tiny_h) {
                // add a random offset.
                Lehmer_rand_gen_64 gen{i};
                auto const angle = angle_dist(gen);

                forces[i] +=
                  force_magnitude * V2f{std::cos(angle), std::sin(angle)};
            } else {
                // Our distances are sorted from least to most,
                // so once we find a distance that's too great we can bail.
                return;
            }
        }
    });
}

void identify_solid_boundaries_and_correct_pressure_forces(
  size_t const particle_count,
  float const support,
  float const world_length,
  float const mass_per_particle,
  Tag* const tags,
  V2f* const pressure_forces,
  V2f const* const positions) {
    float const H = support;
    float const Hp25 = 0.51f * H;
    float const L = world_length;
    float const M = mass_per_particle;
    constexpr float K = 100.0f;

    for_each_iota(particle_count, [=](auto const i) {
        auto tag = tags[i];
        auto const pos_i = positions[i];

        tag.reset(NEAR_SOLID_TAG);
        tag.reset(NEAR_SOLID_VALLEY_TAG);
        tag.reset(NEAR_SOLID_CORNER_TAG);

        V2f pressure_force_correction{0.0f, 0.0f};

        for (int dim = 0; dim < 2; ++dim) {
            // Min wall.
            auto const x_min = Hp25 - pos_i[dim];
            if (x_min > 0.0f) {
                pressure_force_correction[dim] += x_min * M * K / H;

                if (tag.test(NEAR_SOLID_VALLEY_TAG)) {
                    tag.set(NEAR_SOLID_CORNER_TAG);
                } else if (tag.test(NEAR_SOLID_TAG)) {
                    tag.set(NEAR_SOLID_VALLEY_TAG);
                } else {
                    tag.set(NEAR_SOLID_TAG);
                }
            }

            // Max wall.
            auto const x_max = (L - Hp25) - pos_i[dim];
            if (x_max < 0.0f) {
                pressure_force_correction[dim] += x_max * M * K / H;

                if (tag.test(NEAR_SOLID_VALLEY_TAG)) {
                    tag.set(NEAR_SOLID_CORNER_TAG);
                } else if (tag.test(NEAR_SOLID_TAG)) {
                    tag.set(NEAR_SOLID_VALLEY_TAG);
                } else {
                    tag.set(NEAR_SOLID_TAG);
                }
            }
        }

        tags[i] = tag;
        pressure_forces[i] += pressure_force_correction;
    });
}

void enforce_solid_boundaries(size_t const particle_count,
                              float const support,
                              float const world_length,
                              V2f* const positions,
                              V2f* const velocities) {
    float const border = 0.0f;
    V2f const bmin{border, border};
    V2f const bmax{world_length - border, world_length - border};
    for_each_iota(particle_count, [=](auto const i) {
        V2f& p = positions[i];
        V2f& v = velocities[i];

        for (int dim = 0; dim < 2; ++dim) {
            if (p[dim] < bmin[dim]) {
                p[dim] = bmin[dim];
                v[dim] = std::max(0.0f, v[dim]);
            } else if (p[dim] > bmax[dim]) {
                p[dim] = bmax[dim];
                v[dim] = std::min(0.0f, v[dim]);
            }
        }
    });
}

void compute_densities(size_t const particle_count,
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

void update_pressures(size_t const particle_count,
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

void accumulate_pressure_forces(
  size_t const particle_count,
  float const mass_per_particle,
  float const support,
  float const viscosity,
  V2f* const pressure_forces,
  uint8_t const* const neighbor_counts,
  Neighbor_values<size_t> const* const neighbor_indices,
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
        auto const& nbhd_indices = neighbor_indices[i];
        auto const& nbhd_vectors_to = neighbor_vectors_to[i];
        auto const& nbhd_grads_w = neighbor_kernel_gradients[i];
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
                auto const mu_ab = H * dot_vab_rab / (delta_pos_len + N2);
                auto const den_avg = (density + other_density) / 2;

                pressure_force +=
                  (((-alpha * viscosity * mu_ab) + (beta * sqr(mu_ab))) /
                   den_avg) *
                  grad_w;
            }
        }

        pressure_forces[i] += -(M * M) * pressure_force;
    });
}

float max_density_error(size_t const particle_count,
                        float const target_density,
                        float const* const densities) {
    if constexpr (DO_PARALLEL) {
        return tbb::parallel_reduce(
          tbb::blocked_range<float const*>{densities, densities + size},
          0.0f,
          [target_density](tbb::blocked_range<float const*> const& range,
                           float const value) -> float {
              float max_value = value;
              for (auto const density : range) {
                  float const error = std::max(0.0f, density - target_density);
                  max_value = std::max(max_value, error);
              }
              return max_value;
          },
          [](float const a, float const b) -> float { return std::max(a, b); });
    } else {
        float max_error = 0.0f;
        for (size_t i = 0; i < size; ++i) {
            float const error = std::max(0.0f, densities[i] - target_density);
            max_error = std::max(error, max_error);
        }
        return max_error;
    }
}

void compute_colors(size_t const particle_count,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities) {
    for_each_iota(particle_count, [=](auto const i) {
        auto const norm_d = densities[i] / (1.1f * target_density);

        colors[i] = {
          static_cast<uint8_t>(255.0f * std::clamp(1.0f - norm_d, 0.0f, 1.0f)),
          static_cast<uint8_t>(255.0f * std::clamp(1.0f - norm_d, 0.0f, 1.0f)),
          255,
          255};
    });
}

}  // namespace emerald::sph2d_box
