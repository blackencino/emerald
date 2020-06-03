#include <emerald/sph2d_box/simulation.h>

#include <emerald/sph2d_box/solids.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/dynamics.h>
#include <emerald/sph_common/kernels.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

namespace emerald::sph2d_box {

Simulation_config::Simulation_config(Parameters const& in_params)
  : params(in_params) {
    // The grid cells are H*2 on a side, where "H" is support.
    float const H = params.support;
    float const Hhalf = H * 0.5f;
    float const R = 0.99f * Hhalf;
    draw_radius = R;

    // Create a representative, fully-packed volume and calculate the
    // muchness0 and the subsequent per-particle mass and volume.
    // Also use this opportunity to calculate the denominator of the
    // density correction function.
    V2f gradw_sum{0.0f, 0.0f};
    float gradw_dot_gradw_sum = 0.0f;
    float muchness0 = 0.0f;
    float const kernel0 = kernels::W(0.0f, H);
    V2f const pos_i{0.0f * R, 0.588457f * R};

    int num_neighbors = 0;

    // -15 to 15 is more than we need, but just to be safe...
    Box2f const init_volume{{-15.0f * R, -15.0f * R}, {15.0f * R, 15.0f * R}};

    float min_length = 10000.0f;
    V2f min_length_point;
    V2f min_length_delta_position;

    // Fill a cube with points.
    auto const emit_max = estimate_solid_box_emission_count(init_volume, R);
    std::vector<V2f> positions;
    positions.resize(emit_max);
    auto const emitted =
      emit_solid_box(init_volume, R, emit_max, positions.data());
    positions.resize(emitted);

    // Find the point closest to zero.
    V2f best_point;
    float best_r2 = std::numeric_limits<float>::max();
    size_t best_index = emitted + 1;

    for (size_t i = 0; i < emitted; ++i) {
        auto const r2 = positions[i].length2();
        if (r2 < best_r2) {
            best_point = positions[i];
            best_r2 = r2;
            best_index = i;
        }
    }

    for (auto const point : positions) {
        auto const delta_position = best_point - point;
        auto const length = delta_position.length();

        if (length < min_length) {
            min_length = length;
            min_length_point = point;
            min_length_delta_position = delta_position;
        }

        auto const w = kernels::W(length, H);
        if (w > 0.0f) {
            muchness0 += w;
            ++num_neighbors;
        }

        auto const gradw = kernels::GradW(delta_position, H);

        gradw_sum += gradw;
        gradw_dot_gradw_sum += gradw.dot(gradw);
    }

    fmt::print(
      "Min R = {}\n"
      "Min R point = ({}, {})\n"
      "Min R delta_position = ({}, {})\n"
      "num neighbors = {}\n",
      min_length,
      min_length_point[0],
      min_length_point[1],
      min_length_delta_position[0],
      min_length_delta_position[1],
      num_neighbors);

    // Density of a particle is equal to mass times muchness.
    // So density, here, is muchness0 * m_massPerParticle.
    // therefore, m_massPerParticle = kDensityOfWater / muchness0;
    mass_per_particle = params.target_density / muchness0;

    fmt::print(
      "Muchness0: {}\n"
      "Mass per particle: {}\n",
      muchness0,
      mass_per_particle);

    seconds_per_sub_step = to_seconds(params.time_per_step / params.sub_steps);

    // Beta is defined as 2.0 * sqr( dt * m / rho0 );
    // However, we just defined m as rho0/muchness.
    // Therefore, beta is just 2.0 * sqr( dt/muchness );
    // float beta = 2.0 * sqr( dt * m_massPerParticle / kDensityOfWater );
    auto const beta = 2.0f * sqr(seconds_per_sub_step / muchness0);

    // I take out the negative here, and later.
    pressure_correction_denom =
      beta * (gradw_sum.dot(gradw_sum) + gradw_dot_gradw_sum);

    fmt::print(
      "Pressure correction denom: {}\n"
      "gradw_sum dot gradw_sum: {}\n"
      "gradw_sum: {}\n"
      "gradw_dot_gradw_sum: {}\n",
      pressure_correction_denom,
      gradw_sum.dot(gradw_sum),
      gradw_sum,
      gradw_dot_gradw_sum);
};

State dam_break_initial_state(Parameters const& params,
                              Solid_state const& solid_state) {
    State state;

    // The grid cells are H*2 on a side, where "H" is support.
    float const H = params.support;
    float const Hhalf = H * 0.5f;
    float const R = 0.99f * Hhalf;
    float const L = params.length;

    Box2f const init_volume{V2f{0.0f, 0.0f}, V2f{L / 2.0f, 3.25f * L / 4.0f}};

    auto const max_count =
      100 + estimate_solid_box_emission_count(init_volume, R);

    state.positions.resize(max_count);
    auto const emitted_count =
      emit_solid_box(init_volume, R, max_count, state.positions.data());
    state.positions.resize(emitted_count);

    std::vector<uint8_t> kill;
    kill.resize(emitted_count, 0);

    auto const& other_block_map = solid_state.block_map;
    for_each_iota(
      emitted_count,
      [R,
       cell_size = 2.0f * H,
       kill = kill.data(),
       positions = state.positions.data(),
       other_positions = solid_state.positions.data(),
       other_sorted_index_pairs = solid_state.index_pairs.data(),
       &other_block_map](auto const particle_index) {
          auto const pos = positions[particle_index];
          auto const grid_coord =
            compute_grid_coord(pos, V2f{0.0f, 0.0f}, cell_size);
          for (int32_t j = grid_coord[1] - 1; j <= grid_coord[1] + 1; ++j) {
              for (int32_t i = grid_coord[0] - 1; i <= grid_coord[0] + 1; ++i) {
                  auto const other_z_index = z_index::z_index(i, j);
                  auto const found_iter = other_block_map.find(other_z_index);
                  if (found_iter == other_block_map.end()) { continue; }

                  // sip == sorted_index_pair
                  auto const [sip_begin, sip_end] = (*found_iter).second;
                  for (auto sip = sip_begin; sip != sip_end; ++sip) {
                      auto const other_particle_index =
                        other_sorted_index_pairs[sip].second;

                      auto const distance =
                        (other_positions[other_particle_index] - pos).length();
                      if (distance < 2 * R) {
                          kill[particle_index] = 1;
                          return;
                      }
                  }
              }
          }
      });

    std::vector<V2f> surviving_positions;
    surviving_positions.clear();
    surviving_positions.reserve(emitted_count);
    for (size_t i = 0; i < emitted_count; ++i) {
        if (!kill[i]) { surviving_positions.push_back(state.positions[i]); }
    }

    auto const post_kill_count = surviving_positions.size();
    std::swap(state.positions, surviving_positions);

    state.velocities.clear();
    state.velocities.resize(post_kill_count, {0.0f, 0.0f});

    state.colors.clear();
    state.colors.resize(post_kill_count, {128, 128, 255, 255});

    fmt::print("Initial dam break state particle count: {}\n",
               state.positions.size());

    // for (auto const vel : state.velocities) {
    //     fmt::print("Velocity: {}\n", vel);
    // }

    return state;
}

// State random_initial_state(Parameters const& params) {
//     constexpr size_t count = 1000;

//     State state;

//     std::mt19937_64 gen{params.seed};
//     std::uniform_real_distribution<float> pos_dist{0.0f, params.length};
//     state.positions.resize(count);
//     for (auto& [x, y] : state.positions) {
//         x = pos_dist(gen);
//         y = pos_dist(gen);
//     }

//     std::uniform_real_distribution<float> vel_dist{-0.5f * params.length,
//                                                    0.5f * params.length};
//     state.velocities.resize(count);
//     for (auto& [x, y] : state.velocities) {
//         x = vel_dist(gen);
//         y = vel_dist(gen);
//     }

//     state.colors.resize(count, C4uc{128, 128, 255, 255});
//     return state;
// }

void compute_all_neighbhorhoods(Simulation_config const& config,
                                State const& state,
                                Solid_state const& solid_state,
                                Temp_data& temp) {
    auto const count = state.positions.size();
    auto const cell_size = config.params.support * 2.0f;
    temp.grid_coords.resize(count);
    compute_grid_coords(count,
                        cell_size,
                        V2f{0.0f, 0.0f},
                        temp.grid_coords.data(),
                        state.positions.data());

    temp.z_indices.resize(count);
    compute_z_indices(count, temp.z_indices.data(), temp.grid_coords.data());

    temp.index_pairs.resize(count);
    compute_index_pairs(count, temp.index_pairs.data(), temp.z_indices.data());
    sort_index_pairs(count, temp.index_pairs.data());

    temp.block_indices.resize(count);
    compute_block_indices(
      count, temp.block_indices.data(), temp.index_pairs.data());

    size_t const block_count = temp.block_indices.back() + 1;
    temp.blocks.resize(block_count);
    fill_blocks(count, temp.blocks.data(), temp.block_indices.data());

    temp.block_map = create_block_map(block_count,
                                      std::move(temp.block_map),
                                      temp.blocks.data(),
                                      temp.index_pairs.data());

    temp.neighborhood.resize(count);
    create_regular_neighborhoods(count,
                                 cell_size,
                                 temp.neighborhood.counts.data(),
                                 temp.neighborhood.indices.data(),
                                 temp.neighborhood.distances.data(),
                                 temp.neighborhood.vectors_to.data(),
                                 state.positions.data(),
                                 temp.grid_coords.data(),
                                 state.positions.data(),
                                 temp.index_pairs.data(),
                                 temp.block_map);

    temp.solid_neighborhood.resize(count);
    create_regular_neighborhoods(count,
                                 cell_size,
                                 temp.solid_neighborhood.counts.data(),
                                 temp.solid_neighborhood.indices.data(),
                                 temp.solid_neighborhood.distances.data(),
                                 temp.solid_neighborhood.vectors_to.data(),
                                 state.positions.data(),
                                 temp.grid_coords.data(),
                                 solid_state.positions.data(),
                                 solid_state.index_pairs.data(),
                                 solid_state.block_map);
}

void compute_all_neighborhood_kernels(Simulation_config const& config,
                                      Temp_data& temp) {
    auto const count = temp.neighborhood.counts.size();

    temp.neighborhood.kernels.resize(count);
    compute_neighbor_kernels(count,
                             config.params.support,
                             temp.neighborhood.kernels.data(),
                             temp.neighborhood.counts.data(),
                             temp.neighborhood.distances.data());

    temp.neighborhood.kernel_gradients.resize(count);
    compute_neighbor_kernel_gradients(count,
                                      config.params.support,
                                      temp.neighborhood.kernel_gradients.data(),
                                      temp.neighborhood.counts.data(),
                                      temp.neighborhood.vectors_to.data());

    temp.solid_neighborhood.kernels.resize(count);
    compute_neighbor_kernels(count,
                             config.params.support,
                             temp.solid_neighborhood.kernels.data(),
                             temp.solid_neighborhood.counts.data(),
                             temp.solid_neighborhood.distances.data());

    temp.solid_neighborhood.kernel_gradients.resize(count);
    compute_neighbor_kernel_gradients(
      count,
      config.params.support,
      temp.solid_neighborhood.kernel_gradients.data(),
      temp.solid_neighborhood.counts.data(),
      temp.solid_neighborhood.vectors_to.data());
}

void recompute_neighborhood_non_index_values(Simulation_config const& config,
                                             Solid_state const& solid_state,
                                             Temp_data& temp) {
    auto const count = temp.neighborhood.counts.size();
    auto const cell_size = config.params.support * 2.0f;

    compute_neighbor_distances_and_vectors_to(
      count,
      temp.neighborhood.distances.data(),
      temp.neighborhood.vectors_to.data(),
      temp.position_stars.data(),
      temp.position_stars.data(),
      temp.neighborhood.counts.data(),
      temp.neighborhood.indices.data());

    compute_neighbor_distances_and_vectors_to(
      count,
      temp.solid_neighborhood.distances.data(),
      temp.solid_neighborhood.vectors_to.data(),
      temp.position_stars.data(),
      solid_state.positions.data(),
      temp.solid_neighborhood.counts.data(),
      temp.solid_neighborhood.indices.data());

    compute_all_neighborhood_kernels(config, temp);
}

void compute_all_external_forces(Simulation_config const& config,
                                 State const& state,
                                 Temp_data& temp) {
    auto const count = state.positions.size();

    temp.external_forces.resize(count);
    fill_array(count, {0.0f, 0.0f}, temp.external_forces.data());

    // accumulate_constant_pole_attraction_forces(
    //   count,
    //   0.25f * config.params.gravity * config.mass_per_particle,
    //   {0.5f, 0.5f},
    //   temp.external_forces.data(),
    //   state.positions.data());

    accumulate_gravity_forces(count,
                              config.mass_per_particle,
                              config.params.gravity,
                              temp.external_forces.data());

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

void sub_step(Simulation_config const& config,
              State& state,
              Solid_state const& solid_state,
              Temp_data& temp) {
    auto const count = state.positions.size();

    compute_all_neighbhorhoods(config, state, solid_state, temp);
    compute_all_external_forces(config, state, temp);

    // Integrate velocity only from external force
    temp.velocity_external_forces.resize(count);
    copy_array(
      count, temp.velocity_external_forces.data(), state.velocities.data());
    accumulate(count,
               config.seconds_per_sub_step / config.mass_per_particle,
               temp.velocity_external_forces.data(),
               temp.external_forces.data());

    // Init for solving pressures
    temp.pressures.resize(count);
    fill_array(count, 0.0f, temp.pressures.data());

    temp.tags.resize(count);
    temp.pressure_forces.resize(count);
    temp.densities.resize(count);
    temp.position_stars.resize(count);
    temp.velocity_stars.resize(count);

    for (int pci_sub_step = 0; pci_sub_step < 6; ++pci_sub_step) {
        copy_array(count,
                   temp.velocity_stars.data(),
                   temp.velocity_external_forces.data());

        if (pci_sub_step > 0) {
            // integrate velocity
            accumulate(count,
                       config.seconds_per_sub_step / config.mass_per_particle,
                       temp.velocity_stars.data(),
                       temp.pressure_forces.data());
        }

        // integrate position
        copy_array(count, temp.position_stars.data(), state.positions.data());
        accumulate(count,
                   config.seconds_per_sub_step,
                   temp.position_stars.data(),
                   temp.velocity_stars.data());

        // Recompute neighborhood values with kernels.
        recompute_neighborhood_non_index_values(config, solid_state, temp);

        fill_array(count, Tag{}, temp.tags.data());
        fill_array(count, {0.0f, 0.0f}, temp.pressure_forces.data());

        compute_densities(count,
                          config.mass_per_particle,
                          config.params.support,
                          temp.densities.data(),
                          temp.neighborhood.counts.data(),
                          temp.neighborhood.kernels.data());

        accumulate_density_from_solids(count,
                                       config.params.target_density,
                                       temp.densities.data(),
                                       solid_state.volumes.data(),
                                       temp.solid_neighborhood.counts.data(),
                                       temp.solid_neighborhood.indices.data(),
                                       temp.solid_neighborhood.kernels.data());

        update_pressures(count,
                         config.params.target_density,
                         config.pressure_correction_denom,
                         temp.pressures.data(),
                         temp.densities.data());

        accumulate_pressure_forces(count,
                                   config.mass_per_particle,
                                   config.params.support,
                                   config.params.viscosity,
                                   temp.pressure_forces.data(),
                                   temp.neighborhood.counts.data(),
                                   temp.neighborhood.indices.data(),
                                   temp.neighborhood.distances.data(),
                                   temp.neighborhood.vectors_to.data(),
                                   temp.neighborhood.kernel_gradients.data(),
                                   temp.velocity_stars.data(),
                                   temp.pressures.data(),
                                   temp.densities.data());

        accumulate_pressure_forces_from_solids(
          count,
          config.mass_per_particle,
          config.params.target_density,
          temp.pressure_forces.data(),
          solid_state.volumes.data(),
          temp.solid_neighborhood.counts.data(),
          temp.solid_neighborhood.indices.data(),
          temp.solid_neighborhood.kernel_gradients.data(),
          temp.pressures.data(),
          temp.densities.data());

        if (pci_sub_step > 1) {
            auto const max_error = max_density_error(
              count, config.params.target_density, temp.densities.data());
            if ((max_error / config.params.target_density) <= 0.01f) { break; }
        }
    }

    //------------------------------------------------------------------------------
    // FINAL UPDATE

    // Integrate velocity
    copy_array(
      count, state.velocities.data(), temp.velocity_external_forces.data());
    accumulate(count,
               config.seconds_per_sub_step / config.mass_per_particle,
               state.velocities.data(),
               temp.pressure_forces.data());

    // Integrate position
    accumulate(count,
               config.seconds_per_sub_step,
               state.positions.data(),
               state.velocities.data());
}

State simulation_step(Simulation_config const& config,
                      State&& state,
                      Solid_state const& solid_state,
                      Temp_data& temp_data) {
    auto const start = std::chrono::high_resolution_clock::now();

    for (int sub_step_iter = 0; sub_step_iter < config.params.sub_steps;
         ++sub_step_iter) {
        sub_step(config, state, solid_state, temp_data);
    }

    auto const count = state.positions.size();
    state.colors.resize(count);
    compute_colors(count,
                   config.params.target_density,
                   state.colors.data(),
                   temp_data.densities.data());

    auto const end = std::chrono::high_resolution_clock::now();

    fmt::print("PCISPH frame complete, sub_steps: {}, ms: {}\n",
               config.params.sub_steps,
               std::chrono::duration<double>{end - start}.count() * 1000.0);

    return std::move(state);
}

}  // namespace emerald::sph2d_box
