#include <emerald/sph2d_box/dfsph.h>
#include <emerald/sph2d_box/dfsph_p.h>
#include <emerald/sph2d_box/iisph.h>
#include <emerald/sph2d_box/iisph_ap.h>
#include <emerald/sph2d_box/iisph_pseudo_ap.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>

#include <fmt/format.h>

#include <chrono>

int main(int argc, char* argv[]) {
    using namespace emerald::sph2d_box;

    auto const [params, num_batch_frames] = parse_parameters(argc, argv);

    EZ_EXAMPLE_SIM sim{params};
    dfsph_p_init(sim.config, sim.state, sim.solid_state, sim.temp_data);

    for (int frame = 0; frame < num_batch_frames; ++frame) {
        auto const start = std::chrono::high_resolution_clock::now();
        // sim.step();
        // sim.state = dfsph_simulation_step(sim.config, std::move(sim.state),
        //                                   sim.solid_state, sim.temp_data);
        // sim.state = iisph_simulation_step(
        //   sim.config, std::move(sim.state), sim.solid_state, sim.temp_data);
        // sim.state = iisph_ap_simulation_step(
        //   sim.config, std::move(sim.state), sim.solid_state, sim.temp_data);

        sim.state = dfsph_p_simulation_step(
          sim.config, std::move(sim.state), sim.solid_state, sim.temp_data);
        auto const end = std::chrono::high_resolution_clock::now();
        auto const elapsed = end - start;
        fmt::print("Frame: {}, duration: {} ms\n",
                   frame,
                   std::chrono::duration<double, std::milli>{elapsed}.count());
    }
    return 0;
}
