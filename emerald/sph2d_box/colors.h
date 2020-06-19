#pragma once

#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/state.h>
#include <emerald/sph_common/types.h>

#include <functional>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;

using User_colors_function =
  std::function<void(size_t const,          // count
                     float const,           // time
                     float const,           // mass per particle
                     float const,           // radius per particle
                     C4f* const,            // colors
                     V2f const* const,      // positions
                     V2f const* const,      // velocities
                     float const* const)>;  // densities

void target_density_colors(size_t const particle_count,
                           float const target_density,
                           C4f* const colors,
                           float const* const densities);

void compute_colors(float const global_time_in_seconds,
                    Simulation_config const& config,
                    State& state,
                    Solid_state const& solid_state,
                    Temp_data const& temp,
                    User_colors_function const& user_colors);

User_colors_function default_target_density_colors(float const target_density);

void convert_colors(size_t const particle_count,
                    C4uc* const colors_uc,
                    C4f const* const colors_f);

}  // namespace emerald::sph2d_box
