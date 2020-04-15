#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>

#include <fmt/format.h>

#include <chrono>

int main(int argc, char* argv[]) {
    using namespace emerald::sph2d_box;

    auto const [params, num_batch_frames] = parse_parameters(argc, argv);

    EZ_EXAMPLE_SIM sim{params};
    for (int frame = 0; frame < num_batch_frames; ++frame) {
        auto const start = std::chrono::high_resolution_clock::now();
        sim.step();
        auto const end = std::chrono::high_resolution_clock::now();
        auto const elapsed = end - start;
        fmt::print("Frame: {}, duration: {} ms\n",
                   frame, std::chrono::duration<double, std::milli>{elapsed}.count());
    }
    return 0;
}
