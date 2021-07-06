#pragma once

#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/config.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/state.h>
#include <emerald/sph_common/cfl.h>
#include <emerald/sph_common/volume.h>
#include <emerald/util/assert.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>

#include <fmt/format.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace emerald::sph2d_box {

using namespace emerald::sph_common;
using namespace emerald::util;

//------------------------------------------------------------------------------
// Every one of the regular SPH algorithms, excluding PCISPH, has the ability
// to be called with an adaptive timestep. Getting this to work just right is
// difficult, as many of the algorithms are sensitive to very small timesteps
// I don't know why this is the case, and is worthy of investigation, but
// meanwhile... this std adaptive timestep keeps the timesteps in some discrete
// number of sub steps so that the dt is never too small.

template <typename Resize_and_init_function, typename Sub_step_function>
State std_adaptive_time_step(std::optional<std::string> const label,
                             Resize_and_init_function&& resize_and_init,
                             Sub_step_function&& sub_step,
                             int const min_sub_step_denom,
                             int const max_sub_step_denom,
                             float const cfl_factor,
                             flicks const global_time,
                             Simulation_config const& config,
                             State&& state,
                             Solid_state const& solid_state,
                             Temp_data& temp,
                             User_forces_function const& user_forces,
                             User_colors_function const& user_colors) {
    EMLD_ASSERT(min_sub_step_denom % max_sub_step_denom == 0,
                "INVALID SUB_STEP DENOMS. Min denom must be perfectly "
                "divisible by max denom: "
                  << min_sub_step_denom << ", " << max_sub_step_denom);

    auto const start = std::chrono::high_resolution_clock::now();

    auto const particle_count = state.positions.size();
    resize_and_init(particle_count, temp);
    compute_constant_volumes(particle_count,
                             config.params.target_density,
                             config.mass_per_particle,
                             temp.fluid_volumes.data());

    flicks const min_sub_time_step =
      config.params.time_per_step / min_sub_step_denom;
    flicks const max_sub_time_step =
      config.params.time_per_step / max_sub_step_denom;
    auto const sub_step_divisor = min_sub_step_denom / max_sub_step_denom;

    int remaining_sub_steps = min_sub_step_denom;
    flicks const time_per_sub_step =
      config.params.time_per_step / remaining_sub_steps;
    int sub_steps = 0;
    auto time = global_time;
    while (remaining_sub_steps > 0) {
        auto const cfl_step = cfl_maximum_time_step(particle_count,
                                                    config.params.support,
                                                    cfl_factor,
                                                    state.velocities.data());

        auto num_sub_steps = static_cast<int>(
          std::ceil(to_seconds(cfl_step) / to_seconds(time_per_sub_step)));
        num_sub_steps = std::clamp(num_sub_steps, 1, sub_step_divisor);
        num_sub_steps = std::min(num_sub_steps, remaining_sub_steps);

        auto const sub_time_step = num_sub_steps * time_per_sub_step;

        EMLD_ASSERT(
          std::clamp(sub_time_step, min_sub_time_step, max_sub_time_step) ==
            sub_time_step,
          "TIME DISCRETIZATION ERROR");

        sub_step(static_cast<float>(to_seconds(time)),
                 static_cast<float>(to_seconds(sub_time_step)),
                 config,
                 state,
                 solid_state,
                 temp,
                 user_forces);

        remaining_sub_steps -= num_sub_steps;
        ++sub_steps;
        time += sub_time_step;
    }

    compute_colors(static_cast<float>(to_seconds(time)),
                   config,
                   state,
                   solid_state,
                   temp,
                   user_colors);

    auto const end = std::chrono::high_resolution_clock::now();

    if (label.has_value()) {
        fmt::print("{} frame complete, sub_steps: {}, ms: {}\n",
                   label.value(),
                   sub_steps,
                   std::chrono::duration<double>{end - start}.count() * 1000.0);
    }

    return std::move(state);
}

}  // namespace emerald::sph2d_box