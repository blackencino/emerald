#include <emerald/sph2d_box/sim_ops.h>

#include <emerald/sph2d_box/kernels.h>
#include <emerald/util/functions.h>
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

Box2f compute_bounds(size_t const size, V2f const* const positions) {
    if (size < 1) { return Box2f{}; }

    return tbb::parallel_reduce(
        // Range for reduction
        tbb::blocked_range<V2f const*>{positions, positions + size},
        // Identity element
        Box2f{},
        // Reduce a subrange and partial bounds
        [](tbb::blocked_range<V2f const*> const& range, Box2f partial_bounds) {
            for (auto const& p : range) { partial_bounds.extendBy(p); }
            return partial_bounds;
        },
        // Reduce two partial bounds
        [](Box2f bounds_a, Box2f const& bounds_b) {
            bounds_a.extendBy(bounds_b);
            return bounds_a;
        });
}

V2i compute_grid_coord(V2f const p) {
    return {static_cast<int32_t>(std::floor(p[0])),
            static_cast<int32_t>(std::floor(p[1]))};
}

V2i compute_grid_coord(V2f const position, float const cell_size) {
    return compute_grid_coord(position / cell_size);
}

V2i compute_grid_coord(V2f const position,
                       V2f const bounds_min,
                       float const cell_size) {
    return compute_grid_coord(position - bounds_min, cell_size);
}

void compute_grid_coords(size_t const size,
                         float const cell_size,
                         V2f const bounds_min,
                         V2i* const grid_coords,
                         V2f const* const positions) {
    do_parts_work(size, [=](auto const i) {
        grid_coords[i] =
            compute_grid_coord(positions[i], bounds_min, cell_size);
    });
}

void compute_z_indices(size_t const size,
                       uint64_t* const z_indices,
                       V2i const* const grid_coords) {
    do_parts_work(size, [=](auto const i) {
        auto const [x, y] = grid_coords[i];
        z_indices[i] = z_index::z_index(x, y);
    });
}

void compute_index_pairs(size_t const size,
                         std::pair<uint64_t, size_t>* const index_pairs,
                         uint64_t const* const z_indices) {
    do_parts_work(size, [=](auto const i) {
        index_pairs[i] = {z_indices[i], i};
    });
}

void sort_index_pairs(size_t const size,
                      std::pair<uint64_t, size_t>* const index_pairs) {
    if (size < 1) { return; }
    tbb::parallel_sort(index_pairs, index_pairs + size);
}

// This is a weird one. It produces an array where each value is the
// ordered block index, where a block is a storage entity associated with
// each occupied grid cell. So this array will have as many unique values
// as there are unique z indices in the sorted index pairs, and they'll
// be ordered like 000011222223344444455566666677
void compute_block_indices(
    size_t const size,
    uint32_t* const block_indices,
    std::pair<uint64_t, size_t>* const sorted_index_pairs) {
    if (size < 1) { return; }

    // Iterate starting from position 1, init the 0th entry to 0.
    block_indices[0] = 0;

    // Note that we start the range at 1, so we can do i-1.
    tbb::parallel_scan(
        tbb::blocked_range<size_t>{size_t{1}, size},
        0,
        [block_indices, sorted_index_pairs](
            tbb::blocked_range<size_t> const& range,
            uint32_t const sum,
            bool const is_final_scan) {
            uint32_t temp = sum;
            for (auto i = range.begin(); i != range.end(); ++i) {
                if (sorted_index_pairs[i - 1].first !=
                    sorted_index_pairs[i].first) {
                    ++temp;
                }
                if (is_final_scan) { block_indices[i] = temp; }
            }
            return temp;
        },
        [](uint32_t const left, uint32_t const right) { return left + right; });
}

void fill_blocks(size_t const size,
                 std::pair<size_t, size_t>* const blocks,
                 uint32_t* const block_indices) {
    // This one is also weird.
    // The goal is to build each of the blocks, which is a pair
    // of indices representing the begin and end sorted index pair index
    // (ugh what a mouthful) for the particles that live in this block.
    // So what we do is iterate over the block_indices, and if we're at the
    // beginning of a block - that is, if the index of the block_index array
    // is 0, or the block index doesn't match the one before it, we iterate
    // through each position until we see a mismatch, or we reach the end.
    //
    // Most visited entries here will do nothing, but the ones that are at the
    // start of blocks will fill that whole block.
    do_parts_work(size, [=](auto const i) {
        auto const block_index = block_indices[i];
        if (i == 0 || block_indices[i - 1] != block_index) {
            auto& block = blocks[block_index];
            block.first = i;
            for (size_t j = i + 1; j < size; ++j) {
                if (block_indices[j] != block_index) {
                    block.second = j;
                    return;
                }
            }
            // If we get here we went all the way to the end and didn't
            // find a new block, so the end is size
            block.second = size;
        }
    });
}

Block_map create_block_map(
    size_t const block_count,
    Block_map&& into,
    std::pair<size_t, size_t> const* const blocks,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs) {
    into.clear();
    into.rehash(2 * block_count);

    do_parts_work(
        block_count, [&into, blocks, sorted_index_pairs](auto const block_i) {
            auto const block = blocks[block_i];
            auto const z_index = sorted_index_pairs[block.first].first;
            into.emplace(z_index, block);
        });

    return std::move(into);
}

void create_neighborhoods(
    size_t const size,
    float const max_radius,
    Neighborhood* const neighborhoods,
    V2f const* const positions,
    V2i const* const grid_coords,
    std::pair<uint64_t, size_t> const* const sorted_index_pairs,
    Block_map const& block_map) {
    do_parts_work(size, [=](auto const particle_index) {
        auto const pos = positions[particle_index];
        auto const grid_coord = grid_coords[particle_index];
        auto& nbhd = neighborhoods[particle_index];
        nbhd.count = 0;
        for (int32_t j = grid_coord[1] - 1; j <= grid_coord[1] + 1; ++j) {
            for (int32_t i = grid_coord[0] - 1; i <= grid_coord[0] + 1; ++i) {
                auto const other_z_index = z_index::z_index(i, j);
                auto const found_iter = block_map.find(other_z_index);
                if (found_iter == block_map.end()) { continue; }

                auto const [sip_begin, sip_end] = (*found_iter).second;
                for (auto sip = sip_begin; sip != sip_end; ++sip) {
                    auto const other_particle_index =
                        sorted_index_pairs[sip].second;
                    if (other_particle_index == particle_index) { continue; }
                    auto const other_pos = positions[other_particle_index];
                    auto const other_len = (other_pos - pos).length();
                    if (other_len >= max_radius) { continue; }
                    nbhd.indices.at(nbhd.count) =
                        static_cast<uint32_t>(other_particle_index);
                    ++nbhd.count;
                    if (nbhd.count == Neighborhood::MAX_COUNT) {
                        std::sort(nbhd.indices.data(),
                                  nbhd.indices.data() + nbhd.count);
                        return;
                    }
                }
            }
        }
        std::sort(nbhd.indices.data(), nbhd.indices.data() + nbhd.count);
    });
}

void compute_external_forces(size_t const size,
                             float const mass_per_particle,
                             float const gravity,
                             V2f* const forces) {
    fill_array(size, V2f{0.0f, -mass_per_particle * gravity}, forces);
}

void init_pressure(size_t const size, float* const pressures) {
    fill_array(size, 0.0f, pressures);
}

void predict_velocities(size_t const size,
                        float const mass_per_particle,
                        float const dt,
                        V2f* const velocity_stars,
                        V2f const* const velocities,
                        V2f const* const pressure_forces,
                        V2f const* const forces) {
    copy_array(size, velocity_stars, velocities);
    accumulate(size, dt / mass_per_particle, velocity_stars, pressure_forces);
    accumulate(size, dt / mass_per_particle, velocity_stars, forces);
}

void predict_positions(size_t const size,
                       float const dt,
                       V2f* const position_stars,
                       V2f const* const positions,
                       V2f const* const velocities) {
    copy_array(size, position_stars, positions);
    accumulate(size, dt, position_stars, velocities);
}

void reset_tags(size_t const size, Tag* const tags) {
    constexpr Tag empty_tag;
    fill_array(size, empty_tag, tags);
}

void identify_solid_boundaries_and_correct_pressure_forces(
    size_t const size,
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

    do_parts_work(size, [=](auto const i) {
        auto tag = tags[i];
        auto pressure_force = pressure_forces[i];
        auto const pos_i = positions[i];

        tag.reset(NEAR_SOLID_TAG);
        tag.reset(NEAR_SOLID_VALLEY_TAG);
        tag.reset(NEAR_SOLID_CORNER_TAG);

        for (int dim = 0; dim < 2; ++dim) {
            // Min wall.
            auto const x_min = Hp25 - pos_i[dim];
            if (x_min > 0.0f) {
                pressure_force[dim] += x_min * M * K / H;

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
                pressure_force[dim] += x_max * M * K / H;

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
        pressure_forces[i] = pressure_force;
    });
}

void enforce_solid_boundaries(size_t const size,
                              float const support,
                              float const world_length,
                              V2f* const positions,
                              V2f* const velocities) {
    float const border = 0.0f;
    V2f const bmin{border, border};
    V2f const bmax{world_length - border, world_length - border};
    do_parts_work(size, [=](auto const i) {
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

void predict_densities(size_t const size,
                       float const mass_per_particle,
                       float const support,
                       float* const densities,
                       Neighborhood const* const neighborhoods,
                       V2f const* const positions) {
    do_parts_work(size, [=](auto const i) {
        auto const pos_i = positions[i];
        float muchness_predict = kernels::W(0.0f, support);
        auto const& nbhd = neighborhoods[i];
        for (uint32_t nbhd_i = 0; nbhd_i < nbhd.count; ++nbhd_i) {
            auto const j = nbhd.indices[nbhd_i];
            auto const pos_j = positions[j];
            auto const len = (pos_j - pos_i).length();
            if (len >= 2.0f * support) { continue; }
            muchness_predict += kernels::W(len, support);
        }
        densities[i] = mass_per_particle * muchness_predict;
    });
}

void update_pressures(size_t const size,
                      float const target_density,
                      float const pressure_correction_denom,
                      float* const pressures,
                      float const* const densities) {
    do_parts_work(size, [=](auto const i) {
        auto const density_error =
            std::max(densities[i] - target_density, 0.0f);
        auto const pressure_tilda = density_error / pressure_correction_denom;
        pressures[i] += pressure_tilda;
    });
}

void compute_pressure_forces(size_t const size,
                             float const mass_per_particle,
                             float const support,
                             float const viscosity,
                             V2f* const pressure_forces,
                             Neighborhood const* const neighborhoods,
                             V2f const* const positions,
                             V2f const* const velocities,
                             float const* const pressures,
                             float const* const densities) {
    auto const H = support;
    auto const M = mass_per_particle;
    auto const N2 = 0.01f * H * H;
    constexpr float alpha = 1.0f;
    constexpr float beta = 2.0f;

    do_parts_work(size, [=](auto const i) {
        auto const pos_i = positions[i];
        auto const vel_i = velocities[i];
        auto const pressure_i = pressures[i];
        auto const density_i = densities[i];
        float muchness_predict = 0.0f;
        V2f pressure_force = pressure_forces[i];
        auto const& nbhd = neighborhoods[i];
        for (uint32_t nbhd_i = 0; nbhd_i < nbhd.count; ++nbhd_i) {
            auto const j = nbhd.indices[nbhd_i];
            auto const pos_j = positions[j];

            auto delta_pos = pos_i - pos_j;
            auto delta_pos_len = delta_pos.length();
            if (delta_pos_len > 2.0f * H) { continue; }

            auto const vel_j = velocities[j];
            auto const pressure_j = pressures[j];
            auto const density_j = densities[j];

            // Separate couplets
            auto const tiny_h = 0.001f * H;
            if (delta_pos_len < tiny_h) {
                if (j < i) {
                    delta_pos = V2f{0.0f, tiny_h};
                } else {
                    delta_pos = V2f{0.0f, -tiny_h};
                }
                delta_pos_len = tiny_h;
            }

            auto const grad_w = kernels::GradW(delta_pos, H);
            pressure_force += -(M * M) *
                              ((pressure_i / sqr(density_i)) +
                               (pressure_j / sqr(density_j))) *
                              grad_w;

            // Add viscosity.
            // This is a very non-linear force, so it is done in
            // the predictor
            auto const delta_vel = vel_i - vel_j;
            auto const dot_vab_rab = delta_pos.dot(delta_vel);
            if (dot_vab_rab < 0) {
                auto const mu_ab = H * dot_vab_rab / (delta_pos_len + N2);
                auto const den_avg = (density_i + density_j) / 2;

                pressure_force +=
                    -(M * M) *
                    (((-alpha * viscosity * mu_ab) + (beta * sqr(mu_ab))) /
                     den_avg) *
                    grad_w;
            }
        }

        pressure_forces[i] = pressure_force;
    });
}

void update_velocities(size_t const size,
                       float const mass_per_particle,
                       float const dt,
                       V2f* const velocities,
                       V2f const* const pressure_forces,
                       V2f const* const forces) {
    accumulate(size, dt / mass_per_particle, velocities, pressure_forces);
    accumulate(size, dt / mass_per_particle, velocities, forces);
}

void update_positions(size_t const size,
                      float const dt,
                      V2f* const positions,
                      V2f const* const velocities) {
    accumulate(size, dt, positions, velocities);
}

float max_density_error(size_t const size,
                        float const target_density,
                        float const* const densities) {
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
}

void compute_colors(size_t const size,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities) {
    do_parts_work(size, [=](auto const i) {
        auto const norm_d = densities[i] / (1.1f * target_density);

        colors[i] = {static_cast<uint8_t>(
                         255.0f * std::clamp(1.0f - norm_d, 0.0f, 1.0f)),
                     static_cast<uint8_t>(
                         255.0f * std::clamp(1.0f - norm_d, 0.0f, 1.0f)),
                     255,
                     255};
    });
}

}  // namespace emerald::sph2d_box
