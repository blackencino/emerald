#include <emerald/sph2d_box/simulation.h>

#include <emerald/sph2d_box/dfsph_p.h>
#include <emerald/sph2d_box/iisph.h>
#include <emerald/sph2d_box/iisph_ap.h>
#include <emerald/sph2d_box/iisph_pseudo_ap.h>
#include <emerald/sph2d_box/initial_state.h>
#include <emerald/sph2d_box/pcisph.h>
#include <emerald/sph2d_box/solids.h>
#include <emerald/util/assert.h>

namespace emerald::sph2d_box {

EZ_EXAMPLE_SIM::EZ_EXAMPLE_SIM(
  Parameters const& params,
  Solid_initial_state_function const& solid_initial_state,
  Fluid_initial_state_function const& fluid_initial_state,
  User_forces_function in_user_forces,
  User_colors_function in_user_colors)
  : config(params)
  , solid_state(solid_initial_state(params))
  , state(fluid_initial_state(params, solid_state))
  , user_forces(std::move(in_user_forces))
  , user_colors(std::move(in_user_colors))
  , time(0) {
    if (params.method == "pcisph") {
        method = Method::PCISPH;
    } else if (params.method == "iisph") {
        method = Method::IISPH;
    } else if (params.method == "iisph_ap") {
        method = Method::IISPH_AP;
    } else if (params.method == "iisph_pseudo_ap") {
        method = Method::IISPH_PSEUDO_AP;
    } else if (params.method == "dfsph_p") {
        method = Method::DFSPH_P;
    } else {
        EMLD_FAIL("Unknown simulation method: " + params.method);
    }
}

void EZ_EXAMPLE_SIM::step(std::optional<Method> const override_method) {
    // Use the override method or our stored one.
    switch (override_method.value_or(method)) {
    case Method::PCISPH:
        state = pcisph_simulation_step(time,
                                       config,
                                       std::move(state),
                                       solid_state,
                                       temp_data,
                                       user_forces,
                                       user_colors);
        break;
    case Method::IISPH:
        state = iisph_simulation_step(time,
                                      config,
                                      std::move(state),
                                      solid_state,
                                      temp_data,
                                      user_forces,
                                      user_colors);
        break;
    case Method::IISPH_AP:
        state = iisph_ap_simulation_step(time,
                                         config,
                                         std::move(state),
                                         solid_state,
                                         temp_data,
                                         user_forces,
                                         user_colors);
        break;
    case Method::IISPH_PSEUDO_AP:
        state = iisph_pseudo_ap_simulation_step(time,
                                                config,
                                                std::move(state),
                                                solid_state,
                                                temp_data,
                                                user_forces,
                                                user_colors);
        break;
    case Method::DFSPH_P:
    default:
        state = dfsph_p_simulation_step(time,
                                        config,
                                        std::move(state),
                                        solid_state,
                                        temp_data,
                                        user_forces,
                                        user_colors);
        break;
    }

    time += config.params.time_per_step;
}

}  // namespace emerald::sph2d_box
