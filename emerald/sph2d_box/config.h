#pragma once

#include <emerald/sph2d_box/parameters.h>

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

}  // namespace emerald::sph2d_box
