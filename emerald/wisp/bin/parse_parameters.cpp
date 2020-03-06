#include <emerald/wisp/bin/parse_parameters.h>

#include <cxxopts.hpp>

#include <cstdlib>
#include <iostream>

namespace emerald::wisp {

Simulation::Parameters parse_parameters(int argc, char* argv[]) {
    Simulation::Parameters params;
    try {
        // clang-format off
        cxxopts::Options options{argv[0], " - wisp water simulation options"};
        options
            //.allow_unrecognised_options()
            .add_options()
            ("w,world_size", "World size (meters)", cxxopts::value<float>(params.world_size))
            ("r,resolution", "Resolution (pixels), must be power of 2", cxxopts::value<int>(params.resolution))
            ("f,frames_per_second", "Frames per second.", cxxopts::value<float>(params.frames_per_second))
            ("n,num_frames", "Num frames to simulate", cxxopts::value<int>(params.num_batch_frames))
            ("help", "Print help")
            ;
        // clang-format on

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help({""}) << std::endl;
            std::exit(0);
        }
    } catch (const cxxopts::OptionException& e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        std::exit(-1);
    }

    return params;
}

}  // namespace emerald::wisp
