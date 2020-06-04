#pragma once

#include <emerald/sph_common/types.h>

#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph_common {

using namespace emerald::util;

flicks cfl_maximum_time_step(size_t const particle_count,
                             float const support,
                             float const factor,
                             V2f const* const velocities);
}  // namespace emerald::sph_common
