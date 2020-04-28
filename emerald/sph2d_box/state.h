#pragma once

#include <emerald/sph2d_box/foundation.h>
#include <emerald/sph2d_box/sim_ops.h>
#include <emerald/sph2d_box/tag.h>

#include <vector>

namespace emerald::sph2d_box {

struct State {
    std::vector<V2f> positions;
    std::vector<V2f> velocities;
    std::vector<C4uc> colors;
};

struct Temp_data {
    std::vector<V2i> grid_coords;
    std::vector<uint64_t> z_indices;
    std::vector<std::pair<uint64_t, size_t>> index_pairs;
    std::vector<uint32_t> block_indices;
    std::vector<std::pair<size_t, size_t>> blocks;
    Block_map block_map;
    std::vector<Neighborhood> neighborhoods;
    std::vector<Tag> tags;

    std::vector<V2f> forces;
    std::vector<float> pressures;
    std::vector<V2f> pressure_forces;
    std::vector<float> densities;
    std::vector<V2f> position_stars;
    std::vector<V2f> velocity_stars;
};

}  // namespace emerald::sph2d_box