#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/state.h>
#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph2d_box {

State iisph_simulation_step(flicks const global_time,
                            Simulation_config const& config,
                            State&& state,
                            Solid_state const& solid_state,
                            Temp_data& temp,
                            User_forces_function const& user_forces,
                            User_colors_function const& user_colors);

}  // namespace emerald::sph2d_box
