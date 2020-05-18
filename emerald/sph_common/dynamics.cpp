#include <emerald/sph_common/dynamics.h>

namespace emerald::sph_common {

void compute_linear_accelerations_constant_mass(
  size_t const entity_count,
  float const inverse_mass_per_entity,
  V2f* const accelerations,
  V2f const* const forces) {
    accumulate(entity_count, inverse_mass_per_entity, accelerations, forces);
}

void compute_linear_accelerations(size_t const entity_count,
                                  V2f* const accelerations,
                                  float const* const inverse_masses,
                                  V2f const* const forces) {
    for_each_iota(entity_count, [=](auto const entity_index) {
        accelerations[entity_index] +=
          inverse_masses[entity_index] * forces[entity_index];
    });
}

}  // namespace emerald::sph_common
