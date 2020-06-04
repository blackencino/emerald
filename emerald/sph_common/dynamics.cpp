#include <emerald/sph_common/dynamics.h>

namespace emerald::sph_common {

void accumulate_linear_accelerations_constant_mass(
  size_t const entity_count,
  float const inverse_mass_per_entity,
  V2f* const accelerations,
  V2f const* const forces) {
    accumulate(entity_count, inverse_mass_per_entity, accelerations, forces);
}

void accumulate_linear_accelerations(size_t const entity_count,
                                     V2f* const accelerations,
                                     float const* const inverse_masses,
                                     V2f const* const forces) {
    for_each_iota(entity_count, [=](auto const entity_index) {
        accelerations[entity_index] +=
          inverse_masses[entity_index] * forces[entity_index];
    });
}

// Velocities with only external forces
void integrate_velocities_in_place(size_t const particle_count,
                                   float const dt,
                                   float const target_density,
                                   V2f* const velocities,
                                   float const* const volumes,
                                   V2f const* const forces) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        auto const denom = target_density * volumes[particle_index];
        auto const numer = dt * forces[particle_index];
        if (safe_divide(numer, denom)) {
            velocities[particle_index] += numer / denom;
        }
    });
}

void integrate_velocities_in_place(size_t const particle_count,
                                   float const dt,
                                   V2f* const velocities,
                                   V2f const* const accelerations) {
    accumulate(particle_count, dt, velocities, accelerations);
}

void integrate_positions_in_place(size_t const particle_count,
                                  float const dt,
                                  V2f* const positions,
                                  V2f const* const velocities) {
    accumulate(particle_count, dt, positions, velocities);
}

void integrate_velocities_and_positions_in_place(
  size_t const particle_count,
  float const dt,
  V2f* const velocities,
  V2f* const positions,
  V2f const* const accelerations) {
    for_each_iota(particle_count, [=](auto const particle_index) {
        velocities[particle_index] += dt * accelerations[particle_index];
        positions[particle_index] += dt * velocities[particle_index];
    });
}

}  // namespace emerald::sph_common
