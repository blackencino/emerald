#include <emerald/sph2d_box/config.h>

#include <emerald/sph2d_box/solids.h>
#include <emerald/sph_common/kernels.h>
#include <emerald/sph_common/types.h>
#include <emerald/util/flicks.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

#include <fmt/format.h>

#include <limits>

namespace emerald::sph2d_box {

Simulation_config::Simulation_config(Parameters const& in_params)
  : params(in_params) {
    // The grid cells are H*2 on a side, where "H" is support.
    float const H = params.support;
    float const Hhalf = H * 0.5f;
    float const R = 0.99f * Hhalf;
    draw_radius = R;

    // Create a representative, fully-packed volume and calculate the
    // muchness0 and the subsequent per-particle mass and volume.
    // Also use this opportunity to calculate the denominator of the
    // density correction function.
    V2f gradw_sum{0.0f, 0.0f};
    float gradw_dot_gradw_sum = 0.0f;
    float muchness0 = 0.0f;
    float const kernel0 = kernels::W(0.0f, H);
    V2f const pos_i{0.0f * R, 0.588457f * R};

    int num_neighbors = 0;

    // -15 to 15 is more than we need, but just to be safe...
    Box2f const init_volume{{-15.0f * R, -15.0f * R}, {15.0f * R, 15.0f * R}};

    float min_length = 10000.0f;
    V2f min_length_point;
    V2f min_length_delta_position;

    // Fill a cube with points.
    auto const emit_max = estimate_solid_box_emission_count(init_volume, R);
    std::vector<V2f> positions;
    positions.resize(emit_max);
    auto const emitted =
      emit_solid_box(init_volume, R, emit_max, positions.data());
    positions.resize(emitted);

    // Find the point closest to zero.
    V2f best_point;
    float best_r2 = std::numeric_limits<float>::max();
    size_t best_index = emitted + 1;

    for (size_t i = 0; i < emitted; ++i) {
        auto const r2 = positions[i].length2();
        if (r2 < best_r2) {
            best_point = positions[i];
            best_r2 = r2;
            best_index = i;
        }
    }

    for (auto const point : positions) {
        auto const delta_position = best_point - point;
        auto const length = delta_position.length();

        if (length < min_length) {
            min_length = length;
            min_length_point = point;
            min_length_delta_position = delta_position;
        }

        auto const w = kernels::W(length, H);
        if (w > 0.0f) {
            muchness0 += w;
            ++num_neighbors;
        }

        auto const gradw = kernels::GradW(delta_position, H);

        gradw_sum += gradw;
        gradw_dot_gradw_sum += gradw.dot(gradw);
    }

    fmt::print(
      "Min R = {}\n"
      "Min R point = ({}, {})\n"
      "Min R delta_position = ({}, {})\n"
      "num neighbors = {}\n",
      min_length,
      min_length_point[0],
      min_length_point[1],
      min_length_delta_position[0],
      min_length_delta_position[1],
      num_neighbors);

    // Density of a particle is equal to mass times muchness.
    // So density, here, is muchness0 * m_massPerParticle.
    // therefore, m_massPerParticle = kDensityOfWater / muchness0;
    mass_per_particle = params.target_density / muchness0;

    fmt::print(
      "Muchness0: {}\n"
      "Mass per particle: {}\n",
      muchness0,
      mass_per_particle);

    seconds_per_sub_step =
      static_cast<float>(to_seconds(params.time_per_step / params.sub_steps));

    // Beta is defined as 2.0 * sqr( dt * m / rho0 );
    // However, we just defined m as rho0/muchness.
    // Therefore, beta is just 2.0 * sqr( dt/muchness );
    // float beta = 2.0 * sqr( dt * m_massPerParticle / kDensityOfWater );
    auto const beta = 2.0f * sqr(seconds_per_sub_step / muchness0);

    // I take out the negative here, and later.
    pressure_correction_denom =
      beta * (gradw_sum.dot(gradw_sum) + gradw_dot_gradw_sum);

    fmt::print(
      "Pressure correction denom: {}\n"
      "gradw_sum dot gradw_sum: {}\n"
      "gradw_sum: {}\n"
      "gradw_dot_gradw_sum: {}\n",
      pressure_correction_denom,
      gradw_sum.dot(gradw_sum),
      gradw_sum,
      gradw_dot_gradw_sum);
};

}  // namespace emerald::sph2d_box