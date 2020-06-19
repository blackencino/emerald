#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

struct EZ_EXAMPLE_SIM {
    Simulation_config config;
    Solid_state solid_state;
    State state;
    Temp_data temp_data;
    User_forces_function user_forces;
    User_colors_function user_colors;
    flicks time;

    explicit EZ_EXAMPLE_SIM(Parameters const& params);
    void step();
};

}  // namespace emerald::sph2d_box
