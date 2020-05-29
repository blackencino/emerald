#include <emerald/sph2d_box/sim_ops.h>

#include <emerald/sph_common/common.h>
#include <emerald/sph_common/kernels.h>
#include <emerald/util/functions.h>
#include <emerald/util/random.h>
#include <emerald/z_index/z_index.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <utility>

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
        auto const r = positions[i] - pole;
        auto const rN = r.normalized();

        forces[i] -= magnitude * rN;

        // V3f const torque{
        //   0.0f, 0.0f, 0.125f * magnitude /*/ (1.0f + r.dot(r))*/};
        // V3f const r3{rN[0], rN[1], 0.0f};
        // V3f const turn = torque.cross(r3);

        // forces[i] += V2f{turn[0], turn[1]};
    });
}

// This isn't the best drag force, it needs to be conscious of timestep.
void accumulate_simple_drag_forces(size_t const particle_count,
                                   float const magnitude,
                                   float const particle_diameter,
                                   V2f* const forces,
                                   V2f const* const velocities) {
    accumulate(
      particle_count, -magnitude * particle_diameter, forces, velocities);
}

void accumulate_anti_coupling_repulsive_forces(
  size_t const particle_count,
  float const max_distance,
  float const force_magnitude,
  V2f* const forces,
  uint8_t const* const neighbor_counts,
  Neighbor_values<float> const* const neighbor_distances) {
    for_each_iota(particle_count, [=](auto const i) {
        auto const nbhd_count = neighbor_counts[i];
        if (!nbhd_count) { return; }

        auto const& distances = neighbor_distances[i];
        std::uniform_real_distribution<float> angle_dist{float(-M_PI),
                                                         float(M_PI)};

        for (uint8_t j = 0; j < nbhd_count; ++j) {
            if (distances[j] < max_distance) {
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
