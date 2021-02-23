#include <emerald/noise/simplex_noise.h>
#include <emerald/util/format.h>
#include <emerald/util/random.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <cstdint>

namespace emerald::noise {

TEST(Noise_test, SimplexNoised) {
    emerald::util::UniformRand gen{0.0, 1.0};
    SimplexNoised snd;

    for (int i = 0; i < 100; ++i) {
        V3d pt(
          300.0 * (gen() - 0.5), 300.0 * (gen() - 0.5), 300.0 * (gen() - 0.5));

        double t = snd(pt);

        EXPECT_LT(-2.0, t);
        EXPECT_GT(2.0, t);

        fmt::print("Point: {}, noise val: {}\n", pt, t);
    }
}

}  // namespace emerald::noise
