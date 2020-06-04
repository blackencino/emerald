#include <emerald/sph_common/cfl.h>

#include <emerald/sph_common/common.h>
#include <emerald/util/safe_divide.h>

#include <cmath>

namespace emerald::sph_common {

using namespace emerald::util;

flicks cfl_maximum_time_step(size_t const particle_count,
                             float const support,
                             float const factor,
                             V2f const* const velocities) {
    double const numer = factor * support;
    double const denom = std::sqrt(static_cast<double>(
      max_vector_squared_magnitude(particle_count, velocities)));

    return to_flicks(safe_divide(numer, denom).value_or(1.0f));
}

}  // namespace emerald::sph_common