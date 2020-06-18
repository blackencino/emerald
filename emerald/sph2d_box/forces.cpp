#include <emerald/sph2d_box/forces.h>

#include <emerald/sph_common/common.h>
#include <emerald/util/random.h>

#include <cmath>
#include <random>

namespace emerald::sph2d_box {

void compute_all_external_forces(float const dt,
                                 Simulation_config const& config,
                                 State const& state,
                                 Solid_state const& solid_state,
                                 Temp_data& temp,
                                 User_forces_function const& user_forces) {
    auto const count = state.positions.size();

    temp.external_forces.resize(count);
    user_forces(count,
                dt,
                config.mass_per_particle,
                config.draw_radius,
                temp.external_forces.data(),
                state.positions.data(),
                state.velocities.data());

    // accumulate_constant_pole_attraction_forces(
    //   count,
    //   0.25f * config.params.gravity * config.mass_per_particle,
    //   {0.5f, 0.5f},
    //   temp.external_forces.data(),
    //   state.positions.data());

    // accumulate_gravity_forces(count,
    //                           config.mass_per_particle,
    //                           config.params.gravity,
    //                           temp.external_forces.data());

    // accumulate_simple_drag_forces(count,
    //                               0.025f,
    //                               config.params.support,
    //                               temp.external_forces.data(),
    //                               state.velocities.data());

    // // I want an offset of tiny_h
    // // delta_pos = dt * dt / m * f
    // // tiny_h * m / dt * dt
    // auto const tiny_h = 0.001f * config.params.support;
    // auto const magnitude =
    //   config.mass_per_particle * tiny_h / sqr(config.seconds_per_sub_step);
    // accumulate_anti_coupling_repulsive_forces(count,
    //                                           tiny_h,
    //                                           magnitude,
    //                                           temp.external_forces.data(),
    //                                           temp.neighbor_counts.data(),
    //                                           temp.neighbor_distances.data());
}

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

        V3f const torque{
          0.0f, 0.0f, 0.125f * magnitude /*/ (1.0f + r.dot(r))*/};
        V3f const r3{rN[0], rN[1], 0.0f};
        V3f const turn = torque.cross(r3);

        forces[i] += V2f{turn[0], turn[1]};
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

}  // namespace emerald::sph2d_box