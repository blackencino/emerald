#pragma once

#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/state.h>

namespace emerald::sph2d_box {

State dam_break_initial_state(Parameters const& params,
                              Solid_state const& solid_state);

// State random_initial_state(Parameters const& params);

}  // namespace emerald::sph2d_box
