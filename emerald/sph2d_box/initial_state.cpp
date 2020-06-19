#include <emerald/sph2d_box/initial_state.h>

#include <emerald/sph2d_box/solids.h>
#include <emerald/sph_common/common.h>
#include <emerald/sph_common/types.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

#include <algorithm>
#include <vector>

namespace emerald::sph2d_box {

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
    state.colors.resize(post_kill_count, {0.5f, 0.5f, 1.0f, 1.0f});

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

//     state.colors.resize(count, {0.5f, 0.5f, 1.0f, 1.0f});
//     return state;
// }

}  // namespace emerald::sph2d_box
