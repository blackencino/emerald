#include <emerald/sph2d_box/parameters.h>

#include <fmt/format.h>
#include <cxxopts.hpp>

#include <cstdlib>

namespace emerald::sph2d_box {

std::tuple<Parameters, int> parse_parameters(int argc, char* argv[]) {
    Parameters params;
    int num_batch_frames = 100;
    try {
        // clang-format off
        cxxopts::Options options{argv[0], " - sph2d box simulation options"};
        options
            //.allow_unrecognised_options()
            .add_options()
            ("seed", "Random seed", cxxopts::value<uint64_t>(params.seed))
            ("sub_steps", "Sub steps per step", cxxopts::value<int>(params.sub_steps))
            ("length", "Length of world (meters)", cxxopts::value<float>(params.length))
            ("support", "Kernel support (meters)", cxxopts::value<float>(params.support))
            ("gravity", "Gravity (m/s^2)", cxxopts::value<float>(params.gravity))
            ("viscosity", "Viscosity", cxxopts::value<float>(params.viscosity))
            ("density", "Density", cxxopts::value<float>(params.target_density))
            ("n,num_frames", "Num frames to simulate", cxxopts::value<int>(num_batch_frames))
            ("help", "Print help")
            ;
        // clang-format on

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            fmt::print("{}\n", options.help({""}));
            std::exit(0);
        }
    } catch (const cxxopts::OptionException& e) {
        fmt::print(stderr, "Error parsing options: {}\n", e.what());
        std::exit(-1);
    }

    return {params, num_batch_frames};
}

}  // namespace emerald::sph2d_box
