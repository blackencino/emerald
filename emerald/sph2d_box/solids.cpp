#include <emerald/sph2d_box/solids.h>

#include <emerald/util/format.h>

#include <fmt/format.h>

#include <cmath>
#include <random>

namespace emerald::sph2d_box {

// make some points inside some boxes.
size_t estimate_solid_box_emission_count(Box2f const box, float const R) {
    auto const size = box.size();
    auto const num_x = static_cast<size_t>(std::ceil(size[0] / (2.0f * R)));
    auto const num_y =
      static_cast<size_t>(std::ceil(size[1] / (std::sqrt(3.0f) * R)));

    return num_x * num_y;
}

size_t emit_solid_box(Box2f const box,
                      float const R,
                      size_t const max_count,
                      V2f* const positions) {
    bool push_x = false;
    float offset_x = 0.0f;
    float const dx = 2.0f * R;
    float const dy = std::sqrt(3.0f) * R;

    V2f start_point = box.min + V2f{R, R};
    V2f point = start_point;
    V2f stop_max = box.max - V2f{R, R};

    size_t count = 0;
    for (; point.y <= stop_max.y; point.y += dy) {
        // Do a row.
        offset_x = R;
        point.x = push_x ? start_point.x : start_point.x + R;
        point.x += offset_x;
        for (; point.x <= stop_max.x; point.x += dx) {
            positions[count] = point;
            ++count;
            if (count == max_count) { return count; }
        }
        push_x = !push_x;
    }

    return count;
}

void compute_volumes(size_t const particle_count,
                     float const support,
                     float* const volumes,
                     uint8_t const* const neighbor_counts,
                     Neighbor_values<float> const* const neighbor_kernels) {
    auto const muchness_init = kernels::W(0.0f, support);
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const nbhd_count = neighbor_counts[particle_index];
        if (!nbhd_count) {
            volumes[particle_index] = 1.0f / muchness_init;
            return;
        }

        auto const& nbhd_kernels = neighbor_kernels[particle_index];
        float muchness = muchness_init;
        for (uint8_t j = 0; j < nbhd_count; ++j) {
            muchness += nbhd_kernels[j];
        }

        // CJH HACK numer should be 1.0f
        volumes[particle_index] = 1.7f / muchness;
    });
}

Solid_state compute_neighbor_data_and_volumes(Parameters const& params,
                                              Solid_state&& solid_state) {
    auto const count = solid_state.positions.size();
    auto const cell_size = params.support * 2.0f;
    solid_state.grid_coords.resize(count);
    compute_grid_coords(count,
                        cell_size,
                        V2f{0.0f, 0.0f},
                        solid_state.grid_coords.data(),
                        solid_state.positions.data());

    solid_state.z_indices.resize(count);
    compute_z_indices(
      count, solid_state.z_indices.data(), solid_state.grid_coords.data());

    solid_state.index_pairs.resize(count);
    compute_index_pairs(
      count, solid_state.index_pairs.data(), solid_state.z_indices.data());
    sort_index_pairs(count, solid_state.index_pairs.data());

    solid_state.block_indices.resize(count);
    compute_block_indices(
      count, solid_state.block_indices.data(), solid_state.index_pairs.data());

    size_t const block_count = solid_state.block_indices.back() + 1;
    solid_state.blocks.resize(block_count);
    fill_blocks(
      count, solid_state.blocks.data(), solid_state.block_indices.data());

    solid_state.block_map = create_block_map(block_count,
                                             std::move(solid_state.block_map),
                                             solid_state.blocks.data(),
                                             solid_state.index_pairs.data());

    Neighborhood_vectors neighborhood;
    neighborhood.resize(count);
    create_regular_neighborhoods(count,
                                 cell_size,
                                 neighborhood.counts.data(),
                                 neighborhood.indices.data(),
                                 neighborhood.distances.data(),
                                 neighborhood.vectors_to.data(),
                                 solid_state.positions.data(),
                                 solid_state.grid_coords.data(),
                                 solid_state.positions.data(),
                                 solid_state.index_pairs.data(),
                                 solid_state.block_map);
    compute_neighbor_kernels(count,
                             params.support,
                             neighborhood.kernels.data(),
                             neighborhood.counts.data(),
                             neighborhood.distances.data());

    solid_state.volumes.resize(count);
    compute_volumes(count,
                    params.support,
                    solid_state.volumes.data(),
                    neighborhood.counts.data(),
                    neighborhood.kernels.data());

    return std::move(solid_state);
}

Solid_state world_walls_initial_solid_state(Parameters const& params) {
    float const H = params.support;
    float const Hhalf = H * 0.5f;
    float const R = 0.99f * Hhalf;
    float const L = params.length;
    float const border_size = H * 3.0f;

    std::vector<Box2f> boxes;

    // walls
    boxes.emplace_back(V2f{-border_size, -border_size},
                       V2f{0.0f, L + border_size});
    boxes.emplace_back(V2f{L, -border_size},
                       V2f{L + border_size, L + border_size});
    boxes.emplace_back(V2f{-0.5f * border_size, -border_size},
                       V2f{L + 0.5f * border_size, 0.0f});
    boxes.emplace_back(V2f{-0.5f * border_size, L},
                       V2f{L + 0.5f * border_size, L + border_size});

    auto tiny_block = [&boxes, L](V2f const unit, float const roff) {
        float const r = 0.04f + std::abs(roff);
        boxes.emplace_back(L * (unit - V2f{r, r}), L * (unit + V2f{r, r}));
    };

    std::mt19937_64 gen{1919};
    std::uniform_real_distribution<float> dist{-0.06f, 0.06f};

    int const N = 4;
    for (int by = 0; by < N; ++by) {
        float const byf = float(by + 1) / (N+1);

        for (int bx = 0; bx < N; ++bx) {
            float const bxf = float(bx+1) / (N+1);

            // HACK
            tiny_block({bxf + dist(gen), byf + dist(gen)}, dist(gen));
        }
    }


    //for (int i = 0; i < 21; ++i) { tiny_block({dist(gen), dist(gen)}); }

    // tiny_block({0.75f, 0.25f});
    // tiny_block({0.75f, 0.5f});
    // tiny_block({0.75f, 0.75f});
    // tiny_block({0.25f, 0.5f});

    size_t total_estimated_count = 100;
    for (auto const& box : boxes) {
        total_estimated_count += estimate_solid_box_emission_count(box, R);
    }

    Solid_state solid_state;
    solid_state.positions.resize(total_estimated_count);

    auto* emit_at_ptr = solid_state.positions.data();
    auto remaining_count = total_estimated_count;
    for (auto const& box : boxes) {
        auto const emit_count =
          emit_solid_box(box, R, remaining_count, emit_at_ptr);
        remaining_count -= emit_count;
        emit_at_ptr += emit_count;
    }

    auto const emitted_count = total_estimated_count - remaining_count;
    solid_state.positions.resize(emitted_count);

    solid_state.velocities.resize(emitted_count, V2f{0.0f, 0.0f});
    solid_state.colors.resize(emitted_count, C4uc{200, 155, 20, 255});

    // do neighborhoods and volumes.
    solid_state =
      compute_neighbor_data_and_volumes(params, std::move(solid_state));

    return solid_state;
}

void accumulate_density_from_solids(
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

void accumulate_pressure_forces_from_solids(
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
