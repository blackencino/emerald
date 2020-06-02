#pragma once

#include <emerald/sph2d_box/iisph.h>
#include <emerald/sph2d_box/iisph_pseudo_ap.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/sph2d_box/state.h>

#include <emerald/util/flicks.h>

#include <cstdint>

namespace emerald::sph2d_box {

void iisph_pseudo_ap_sub_step(float const dt,
                              Simulation_config const& config,
                              State& state,
                              Solid_state const& solid_state,
                              Temp_data& temp);

State iisph_pseudo_ap_simulation_step(Simulation_config const& config,
                                      State&& state,
                                      const Solid_state& solid_state,
                                      Temp_data& temp);

}  // namespace emerald::sph2d_box
