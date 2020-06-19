#include <emerald/sph2d_box/colors.h>

#include <emerald/sph_common/common.h>

#include <algorithm>

namespace emerald::sph2d_box {

void target_density_colors(size_t const particle_count,
                           float const target_density,
                           C4f* const colors,
                           float const* const densities) {
    for_each_iota(particle_count, [=](auto const i) {
        auto const norm_d = densities[i] / (1.1f * target_density);

        auto const c = std::clamp(1.0f - norm_d, 0.0f, 1.0f);

        colors[i] = {c, c, 1.0f, 1.0f};
    });
}

void compute_colors(float const global_time_in_seconds,
                    Simulation_config const& config,
                    State& state,
                    Solid_state const& solid_state,
                    Temp_data const& temp,
                    User_colors_function const& user_colors) {
    auto const count = state.positions.size();
    state.colors.resize(count);
    user_colors(count,
                global_time_in_seconds,
                config.mass_per_particle,
                config.draw_radius,
                state.colors.data(),
                state.positions.data(),
                state.velocities.data(),
                temp.densities.data());
}

User_colors_function default_target_density_colors(float const target_density) {
    return [target_density](size_t const count,  // count
                            float const,         // time
                            float const,         // mass per particle
                            float const,         // radius per particle
                            C4f* const colors,   // colors
                            V2f const* const,    // positions
                            V2f const* const,    // velocities
                            float const* const densities) {
        target_density_colors(count, target_density, colors, densities);
    };
}

static uint8_t convert_channel(float const f) {
    return static_cast<uint8_t>(255.0f * std::clamp(f, 0.0f, 1.0f));
}

static C4uc convert_pixel(C4f const& c) {
    return {convert_channel(c[0]),
            convert_channel(c[1]),
            convert_channel(c[2]),
            convert_channel(c[3])};
}

void convert_colors(size_t const count,
                    C4uc* const colors_uc,
                    C4f const* const colors_f) {
    for_each_iota(count, [=](auto const particle_index) {
        colors_uc[particle_index] = convert_pixel(colors_f[particle_index]);
    });
}

}  // namespace emerald::sph2d_box
