#pragma once

#include <emerald/sph_common/common.h>
#include <emerald/sph_common/types.h>

namespace emerald::sph_common {

void compute_linear_accelerations_constant_mass(
  size_t const entity_count,
  float const inverse_mass_per_entity,
  V2f* const accelerations,
  V2f const* const forces);

void compute_linear_accelerations(size_t const entity_count,
                                  V2f* const accelerations,
                                  float const* const inverse_masses,
                                  V2f const* const forces);

template <typename T>
void forward_euler_integrate_linear_values(size_t const entity_count,
                                           float const dt,
                                           T* const value_nexts,
                                           T const* const values,
                                           T const* const value_dots) {
    // Accomodate in-place
    if (values != value_nexts) { copy_array(entity_count, value_nexts, values); }

    accumulate(entity_count, dt, value_nexts, value_dots);
}

}  // namespace emerald::sph_common
