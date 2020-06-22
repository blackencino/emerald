#include <emerald/sph2d_box/simulation.h>

#include <emerald/sph2d_box/initial_state.h>
#include <emerald/sph2d_box/pcisph.h>
#include <emerald/sph2d_box/solids.h>

namespace emerald::sph2d_box {

EZ_EXAMPLE_SIM::EZ_EXAMPLE_SIM(Parameters const& params)
  : config(params)
  , solid_state(world_walls_initial_solid_state(params))
  , state(dam_break_initial_state(params, solid_state))
  , user_forces(default_gravity_forces(config.params.gravity))
  , user_colors(default_target_density_colors(config.params.target_density))
  , time(0) {
}

void EZ_EXAMPLE_SIM::step() {
    state = pcisph_simulation_step(time,
                                   config,
                                   std::move(state),
                                   solid_state,
                                   temp_data,
                                   user_forces,
                                   user_colors);
    time += config.params.time_per_step;
}

}  // namespace emerald::sph2d_box
