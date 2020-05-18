#include <emerald/util/format.h>
#include <emerald/util/foundation.h>
#include <emerald/util/random.h>
#include <emerald/util/vector_util.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace emerald::util {

TEST(Test_vector_util, Print_hash_key) {
    V3d g;
    set_zero(g);

    UniformRand rand;
    std::vector<float> v;
    for (int i = 0; i < 100; ++i) { v.push_back(rand()); }

    auto const* const vdata = v.data();
    for (int i = 0; i < 100; ++i) { fmt::print("{}\n", vdata[i]); }

    auto const vkey = ComputeVectorHashKey(v);
    fmt::print("Vector hash key: {}\n", FormatHashKey(vkey));
}

}  // namespace emerald::util