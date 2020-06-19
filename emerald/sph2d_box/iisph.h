#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/sph2d_box/state.h>

#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph2d_box {

void iisph_sub_step(float const dt,
                    Simulation_config const& config,
                    State& state,
                    Solid_state const& solid_state,
                    Temp_data& temp);

void iisph_resize_temp_arrays(size_t const particle_count, Temp_data& temp);

State iisph_simulation_step(Simulation_config const& config,
                            State&& state,
                            Solid_state const& solid_state,
                            Temp_data& temp);

}  // namespace emerald::sph2d_box
