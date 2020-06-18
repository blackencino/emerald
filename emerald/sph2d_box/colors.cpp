#include <emerald/sph2d_box/colors.h>

#include <emerald/sph_common/common.h>

#include <algorithm>

namespace emerald::sph2d_box {

void compute_colors(size_t const particle_count,
                    float const target_density,
                    C4uc* const colors,
                    float const* const densities) {
    for_each_iota(particle_count, [=](auto const i) {
        auto const norm_d = densities[i] / (1.1f * target_density);

        colors[i] = {
          static_cast<uint8_t>(255.0f * std::clamp(1.0f - norm_d, 0.0f, 1.0f)),
          static_cast<uint8_t>(255.0f * std::clamp(1.0f - norm_d, 0.0f, 1.0f)),
          255,
          255};
    });
}

}  // namespace emerald::sph2d_box
