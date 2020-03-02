#include <emerald/shallow_weaver/bin/parse_parameters.h>

#include <cxxopts.hpp>

#include <cstdlib>
#include <iostream>

namespace emerald::shallow_weaver {

Simulation::Parameters parse_parameters(int argc, char* argv[]) {
    Simulation::Parameters params;
    try {
        // clang-format off
        cxxopts::Options options{argv[0], " - shallow water simulation options"};
        options
            //.allow_unrecognised_options()
            .add_options()
            ("w,world_size", "World size (meters)", cxxopts::value<float>(params.world_size))
            ("r,resolution", "Resolution (pixels), must be power of 2", cxxopts::value<int>(params.resolution))
            ("f,frames_per_second", "Frames per second.", cxxopts::value<float>(params.frames_per_second))
            ("n,num_frames", "Num frames to simulate", cxxopts::value<int>(params.num_batch_frames))
            ("wave_speed", "Wave speed (meters per second)", cxxopts::value<float>(params.wave_speed))
            ("damping", "Damping (per second)", cxxopts::value<float>(params.damping)
             ->default_value(std::to_string(params.damping)))
            ("rk2", "Use Runge-Kutta-2 time integration")
            ("rk4", "Use Runge-Kutta-4 time integration")
            ("help", "Print help")
            ;
        // clang-format on

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help({""}) << std::endl;
            std::exit(0);
        }

        if (result.count("rk2")) {
            params.time_integration = Time_integration::RUNGE_KUTTA_2;
        }

        if (result.count("rk4")) {
            params.time_integration = Time_integration::RUNGE_KUTTA_4;
        }
    } catch (const cxxopts::OptionException& e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        std::exit(-1);
    }

    return params;
}

}  // namespace emerald::shallow_weaver
