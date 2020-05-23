#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/sph2d_box/state.h>

#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph2d_box {

flicks cfl_maximum_time_step(size_t const particle_count,
                             float const support,
                             V2f const* const velocities);

void dfsph_sub_step(flicks const delta_time,
                    Simulation_config const& config,
                    State& state,
                    const Solid_state& solid_state,
                    Temp_data& temp);

State dfsph_simulation_step(Simulation_config const& config,
                            State&& state,
                            const Solid_state& solid_state,
                            Temp_data& temp_data);

}  // namespace emerald::sph2d_box
