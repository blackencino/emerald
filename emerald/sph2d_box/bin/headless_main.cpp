#include <emerald/sph2d_box/colors.h>
#include <emerald/sph2d_box/forces.h>
#include <emerald/sph2d_box/initial_state.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/sph2d_box/solids.h>

#include <fmt/format.h>

#include <chrono>

int main(int argc, char* argv[]) {
    using namespace emerald::sph2d_box;

    auto const params = parse_parameters(argc, argv);

    EZ_EXAMPLE_SIM sim{params,
                       world_walls_initial_solid_state,
                       dam_break_initial_state,
                       default_gravity_forces(params.gravity),
                       default_target_density_colors(params.target_density)};
    for (int step = 0; step < params.num_batch_steps; ++step) {
        auto const start = std::chrono::high_resolution_clock::now();
        sim.step();
        auto const end = std::chrono::high_resolution_clock::now();
        auto const elapsed = end - start;
        fmt::print("{} Step: {}, duration: {} ms\n",
                   params.method,
                   step,
                   std::chrono::duration<double, std::milli>{elapsed}.count());
    }
    return 0;
}
