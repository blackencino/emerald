#include <emerald/sph2d_box/iisph_ap_ops.h>

namespace emerald::sph2d_box {


void iisph_ap_compute_diagonals(...) {

    for_each_iota(... [=](auto const particle_index) {

        V2f sum_mf_gradwif{0.0f, 0.0f};
        float sum_mf_gradwif_dot_gradwif = 0.0f;

        // Loop over fluids
        for (uint8_t j = 0; j < fluid_nbhd_count; ++j) {
            auto const neighbor_particle_index = ;

            grad_w = ;
            mass = ;
            sum_mf_gradwif += mass * grad_w;

            sum_mf_gradwif_dot_gradwif += mass * grad_w.dot(grad_w);
        }

        // Loop over solids
        V2f sum_mb_gradwib{0.0f, 0.0f};

        for (uint8_t j = 0; j < solid_nbhd_count; ++j) {
            auto const neighbor_particle_index = ;

            grad_w = ;
            mass = ;
            sum_mb_gradwib += mass * grad_w;
        }

        aii = (sum_mf_gradwif + sum_mb_gradwib).dot(
                                                    sum_mf_gradwif/sqr(density) +
                                                    sum_mb_gradwib/sqr(density) +
                                                    sum_mb_gradwib/sqr(target_density)) +
        mass * sum_mf_gradwif_dot_gradwif / sqr(density);

        diagonals[particle_index] = -sqr(dt) * aii;
    });
}

} // namespace