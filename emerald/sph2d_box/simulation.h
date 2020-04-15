#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

struct Simulation_config {
    Parameters params;
    float seconds_per_sub_step = 1.0f;
    float mass_per_particle = 1.0f;
    float draw_radius = 1.0f;

    // This is very PCI-SPH-specific
    float pressure_correction_denom = 1.0f;

    Simulation_config() = default;
    explicit Simulation_config(Parameters const& in_params);
};

State dam_break_initial_state(Parameters const& params);

State simulation_step(Simulation_config const& config,
                      State&& state,
                      Temp_data& temp_data);

struct EZ_EXAMPLE_SIM {
    Simulation_config config;
    State state;
    Temp_data temp_data;

    explicit EZ_EXAMPLE_SIM(Parameters const& params)
      : config(params)
      , state(dam_break_initial_state(params)) {
    }

    void step() {
        state = simulation_step(config, std::move(state), temp_data);
    }
};

}  // namespace emerald::sph2d_box
