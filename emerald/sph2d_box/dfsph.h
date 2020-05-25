#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/sph2d_box/state.h>

#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph2d_box {

flicks dfsph_cfl_maximum_time_step(size_t const particle_count,
                                   float const support,
                                   V2f const* const velocities);

void dfsph_integrate_velocity_from_non_pressure_forces(
  float const dt,
  Simulation_config const& config,
  State& state,
  Solid_state const& solid_state,
  Temp_data& temp);

void dfsph_integrate_positions(float const dt,
                               Simulation_config const& config,
                               State& state,
                               Solid_state const& solid_state,
                               Temp_data& temp);

void dfsph_compute_alphas(Simulation_config const& config,
                          State& state,
                          Solid_state const& solid_state,
                          Temp_data& temp);

float dfsph_compute_divergence_error_kappas(float const dt,
                                            Simulation_config const& config,
                                            State& state,
                                            Solid_state const& solid_state,
                                            Temp_data& temp);

void dfsph_apply_kappas(float const dt,
                        float const* const kappas,
                        Simulation_config const& config,
                        State& state,
                        Solid_state const& solid_state,
                        Temp_data& temp);

void dfsph_compute_densities(float* const densities,
                             Simulation_config const& config,
                             State& state,
                             Solid_state const& solid_state,
                             Temp_data& temp);

void dfsph_correct_divergence_error(float const dt,
                                    Simulation_config const& config,
                                    State& state,
                                    Solid_state const& solid_state,
                                    Temp_data& temp);

void dfsph_correct_density_error(float const dt,
                                 Simulation_config const& config,
                                 State& state,
                                 Solid_state const& solid_state,
                                 Temp_data& temp);

void dfsph_sub_step(float const dt,
                    Simulation_config const& config,
                    State& state,
                    Solid_state const& solid_state,
                    Temp_data& temp);

void dfsph_resize_and_init_temp_arrays(size_t const particle_count,
                                       Temp_data& temp);

State dfsph_simulation_step(Simulation_config const& config,
                            State&& state,
                            const Solid_state& solid_state,
                            Temp_data& temp_data);

}  // namespace emerald::sph2d_box
