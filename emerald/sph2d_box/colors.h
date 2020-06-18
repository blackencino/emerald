#pragma once

#include <emerald/common/types.h>

#include <functional>

namespace emerald::sph2d_box {

using User_colors_function =
  std::function<void(size_t const,          // count
                     float const,           // dt
                     float const,           // mass per particle
                     float const,           // radius per particle
                     C4f* const,            // colors
                     V2f const* const,      // positions
                     V2f const* const,      // velocities
                     float const* const)>;  // densities

void compute_colors(size_t const particle_count,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities);

}  // namespace emerald::sph2d_box
