#pragma once

#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/solids.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {


void sub_step(Simulation_config const& config,
              State& state,
              Solid_state const& solid_state,
              Temp_data& temp);

State simulation_step(Simulation_config const& config,
                      State&& state,
                      Solid_state const& solid_state,
                      Temp_data& temp_data);

struct EZ_EXAMPLE_SIM {
    Simulation_config config;
    Solid_state solid_state;
    State state;
    Temp_data temp_data;

    explicit EZ_EXAMPLE_SIM(Parameters const& params)
      : config(params)
      , solid_state(world_walls_initial_solid_state(params))
      , state(dam_break_initial_state(params, solid_state)) {
    }

    void step() {
        state =
          simulation_step(config, std::move(state), solid_state, temp_data);
    }
};

}  // namespace emerald::sph2d_box
