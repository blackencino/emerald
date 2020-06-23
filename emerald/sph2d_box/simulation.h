#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/initial_state.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/solids.h>
#include <emerald/sph2d_box/state.h>

#include <optional>

namespace emerald::sph2d_box {

//------------------------------------------------------------------------------
// The goal of this class is not to be a simulation framework top,
// but rather to demonstrate how the simulation tools might be used.
// There are lots of different ways to calculate initial state,
// and lots of different ways to update the state. For example, a
// simulator running in a gRPC environment will want to copy the state
// with each timestep so that it can communicate the results while it
// has the next iteration computed in a separate thread or threads.

struct EZ_EXAMPLE_SIM {
    enum Method { PCISPH, IISPH, IISPH_AP, IISPH_PSEUDO_AP, DFSPH_P };

    Simulation_config config;
    Solid_state solid_state;
    State state;
    Temp_data temp_data;
    User_forces_function user_forces;
    User_colors_function user_colors;
    flicks time = flicks{0};
    Method method = Method::IISPH_PSEUDO_AP;

    explicit EZ_EXAMPLE_SIM(
      Parameters const& params,
      Solid_initial_state_function const& solid_initial_state,
      Fluid_initial_state_function const& fluid_initial_state,
      User_forces_function user_forces,
      User_colors_function user_colors);

    void step(std::optional<Method> const method_override = std::nullopt);
};

}  // namespace emerald::sph2d_box
