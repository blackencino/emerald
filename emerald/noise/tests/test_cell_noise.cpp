#include <emerald/noise/cell_noise.h>
#include <emerald/util/format.h>
#include <emerald/util/random.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <cstdint>

namespace emerald::noise {

TEST(Cell_noise_test, CellNoised) {
    for (float f = 0.1f; f <= 100.0f; f += 0.371773f) {
        Imath::V3f const pt{f, 0.0f, 0.0f};
        auto const v = CellNoise3(pt);

        EXPECT_LT(-2.0, v.x);
        EXPECT_GT(2.0, v.x);

        EXPECT_LT(-2.0, v.y);
        EXPECT_GT(2.0, v.y);

        EXPECT_LT(-2.0, v.z);
        EXPECT_GT(2.0, v.z);

        fmt::print("Point: {}, cell noise val: {}\n", pt, v);
    }
}

}  // namespace emerald::noise
