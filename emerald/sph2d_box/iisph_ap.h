#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

State iisph_ap_simulation_step(flicks const time,
                               Simulation_config const& config,
                               State&& state,
                               Solid_state const& solid_state,
                               Temp_data& temp,
                               User_forces_function const& user_forces,
                               User_colors_function const& user_colors);

}  // namespace emerald::sph2d_box
