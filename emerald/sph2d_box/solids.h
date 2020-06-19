#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

size_t estimate_solid_box_emission_count(Box2f const box, float const R);

size_t emit_solid_box(Box2f const box,
                      float const R,
                      size_t const max_count,
                      V2f* const positions);

void compute_volumes(size_t const particle_count,
                     float const support,
                     float* const volumes,
                     uint8_t const* const neighbor_counts,
                     Neighbor_values<float> const* const neighbor_kernels);

Solid_state compute_neighbor_data_and_volumes(Parameters const& params,
                                              Solid_state&& solid_state);

Solid_state world_walls_initial_solid_state(Parameters const& params);

}  // namespace emerald::sph2d_box