#pragma once

#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

void compute_all_neighbhorhoods(Simulation_config const& config,
                                State const& state,
                                Solid_state const& solid_state,
                                Temp_data& temp);

void compute_all_neighborhood_kernels(Simulation_config const& config,
                                      Temp_data& temp);

void recompute_neighborhood_non_index_values(Simulation_config const& config,
                                             Solid_state const& solid_state,
                                             Temp_data& temp);

}  // namespace emerald::sph2d_box