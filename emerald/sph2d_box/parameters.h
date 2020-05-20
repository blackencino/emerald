#pragma once

#include <emerald/util/flicks.h>

#include <cstdint>
#include <tuple>

namespace emerald::sph2d_box {

using namespace emerald::util;

struct Parameters {
    uint64_t seed = 1;
    flicks time_per_step = 3 * k_flicks_one_ninetieth_of_second;
    int sub_steps = 30;
    float length = 1.0f;
    float support = 0.025f;
    float gravity = 9.81f;
    float viscosity = 0.01f;
    float target_density = 1000.0f;
};

std::tuple<Parameters, int> parse_parameters(int argc, char* argv[]);

}  // namespace emerald::sph2d_box
