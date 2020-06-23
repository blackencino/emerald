#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

#include <functional>

namespace emerald::sph2d_box {

using Fluid_initial_state_function =
  std::function<State(Parameters const&, Solid_state const&)>;

State dam_break_initial_state(Parameters const& params,
                              Solid_state const& solid_state);

}  // namespace emerald::sph2d_box
