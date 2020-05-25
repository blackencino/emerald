#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/solids.h>
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

State dam_break_initial_state(Parameters const& params,
                              Solid_state const& solid_state);

// State random_initial_state(Parameters const& params);

void compute_all_neighbhorhoods(Simulation_config const& config,
                                State const& state,
                                Solid_state const& solid_state,
                                Temp_data& temp);

void compute_all_neighborhood_kernels(Simulation_config const& config,
                                      Temp_data& temp);

void recompute_neighborhood_non_index_values(Simulation_config const& config,
                                             Solid_state const& solid_state,
                                             Temp_data& temp);

void compute_all_external_forces(Simulation_config const& config,
                                 State const& state,
                                 Temp_data& temp);

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
