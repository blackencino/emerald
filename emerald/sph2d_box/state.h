#pragma once

#include <emerald/sph2d_box/sim_ops.h>
#include <emerald/sph2d_box/tag.h>
#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

#include <cstdint>
#include <vector>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

struct State {
    std::vector<V2f> positions;
    std::vector<V2f> velocities;
    std::vector<C4uc> colors;
};

struct Solid_state {
    std::vector<V2f> positions;
    std::vector<V2f> velocities;
    std::vector<C4uc> colors;

    std::vector<V2i> grid_coords;
    std::vector<uint64_t> z_indices;
    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    std::vector<uint32_t> block_indices;
    std::vector<std::pair<size_t, size_t>> blocks;
    Block_map block_map;

    std::vector<float> volumes;
};

struct Temp_data {
    std::vector<V2i> grid_coords;
    std::vector<uint64_t> z_indices;
    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    std::vector<uint32_t> block_indices;
    std::vector<std::pair<size_t, size_t>> blocks;
    Block_map block_map;

    Neighborhood_vectors neighborhood;

    std::vector<Tag> tags;

    std::vector<V2f> external_forces;
    std::vector<float> pressures;
    std::vector<V2f> pressure_forces;
    std::vector<float> densities;

    std::vector<V2f> velocity_external_forces;

    std::vector<V2f> position_stars;
    std::vector<V2f> velocity_stars;

    Neighborhood_vectors solid_neighborhood;

    std::vector<V3f> alpha_denom_parts;
    std::vector<float> alphas;
    std::vector<float> divergence_kappas;
    std::vector<float> density_kappas;
    std::vector<float> density_stars;

    std::vector<float> aiis;
    std::vector<V2f> diis;
    std::vector<V2f> sum_dij_pjs;
    std::vector<float> fluid_volumes;
    std::vector<float> new_pressures;
};

}  // namespace emerald::sph2d_box