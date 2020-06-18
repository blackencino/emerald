#include <emerald/sph2d_box/neighborhoods_from_state.h>

#include <emerald/sph_common/common.h>
#include <emerald/sph_common/neighborhood.h>
#include <emerald/sph_common/types.h>

namespace emerald::sph2d_box {

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

}  // namespace emerald::sph2d_box
