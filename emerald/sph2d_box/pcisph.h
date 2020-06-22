#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/solids.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

void pcisph_sub_step(float const global_time_in_seconds,
                     float const dt,
                     Simulation_config const& config,
                     State& state,
                     Solid_state const& solid_state,
                     Temp_data& temp,
                     User_forces_function const& user_forces);

State pcisph_simulation_step(flicks const global_time,
                             Simulation_config const& config,
                             State&& state,
                             Solid_state const& solid_state,
                             Temp_data& temp_data,
                             User_forces_function const& user_forces,
                             User_colors_function const& user_colors);

}  // namespace emerald::sph2d_box
