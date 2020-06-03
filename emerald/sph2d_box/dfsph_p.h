#pragma once

#include <emerald/sph2d_box/iisph.h>
#include <emerald/sph2d_box/iisph_ap.h>
#include <emerald/sph2d_box/iisph_pseudo_ap.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/sph2d_box/state.h>

#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph2d_box {

void dfsph_p_correct_density_error(float const dt,
                                   Simulation_config const& config,
                                   State& state,
                                   Solid_state const& solid_state,
                                   Temp_data& temp);

void dfsph_p_correct_divergence_error(float const dt,
                                      Simulation_config const& config,
                                      State& state,
                                      Solid_state const& solid_state,
                                      Temp_data& temp);

void dfsph_p_resize_and_init_temp_arrays(size_t const particle_count,
                                         Temp_data& temp);

void dfsph_p_init(Simulation_config const& config,
                  State const& state,
                  const Solid_state& solid_state,
                  Temp_data& temp);

void dfsph_p_sub_step(float const dt,
                      Simulation_config const& config,
                      State& state,
                      Solid_state const& solid_state,
                      Temp_data& temp);

State dfsph_p_simulation_step(Simulation_config const& config,
                              State&& state,
                              const Solid_state& solid_state,
                              Temp_data& temp);

}  // namespace emerald::sph2d_box