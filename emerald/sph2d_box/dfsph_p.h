#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/parameters.h>
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
                  Solid_state const& solid_state,
                  Temp_data& temp);

void dfsph_p_sub_step(float const global_time_in_seconds,
                      float const dt,
                      Simulation_config const& config,
                      State& state,
                      Solid_state const& solid_state,
                      Temp_data& temp,
                      User_forces_function const& user_forces);

State dfsph_p_simulation_step(flicks const global_time,
                              Simulation_config const& config,
                              State&& state,
                              Solid_state const& solid_state,
                              Temp_data& temp,
                              User_forces_function const& user_forces,
                              User_colors_function const& colors);

}  // namespace emerald::sph2d_box