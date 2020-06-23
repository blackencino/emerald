#pragma once

#include <emerald/util/flicks.h>

#include <cstdint>
#include <string>

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

    struct {
        int max_pressure_iterations = 30;
        float error_average_threshold = 0.005f;
        float error_max_threshold = 0.05f;
        float omega = 0.5f;
    } iisph;

    struct {
        int max_correction_iterations = 15;

        float density_error_average_threshold = 0.0035f;
        float density_error_max_threshold = 0.03f;

        float divergence_error_average_threshold = 0.0035f;
        float divergence_error_max_threshold = 0.03f;
    } dfsph;

    int num_batch_steps = 100;
    std::string method = "iisph_pseudo_ap";
};

Parameters parse_parameters(int argc, char* argv[]);

}  // namespace emerald::sph2d_box
