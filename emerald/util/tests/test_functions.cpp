#include <emerald/util/functions.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <typeinfo>

namespace emerald::util {

//------------------------------------------------------------------------------
template <typename T>
void testWrapT(T const x,
               T const lb,
               T const ub,
               T const expected,
               [[maybe_unused]] T const tol) {
    auto const k = wrap<T>(x, lb, ub);
    fmt::print("wrap<{}>({}, {}, {}) = {}, expecting: {}\n",
               typeid(T).name(),
               x,
               lb,
               ub,
               k,
               expected);
    if constexpr (std::is_integral_v<T>) {
        EXPECT_EQ(expected, k);
    } else {
        EXPECT_NEAR(expected, k, tol);
    }
}

//------------------------------------------------------------------------------
TEST(Test_functions, Test_wrap_integers) {
    testWrapT<int>(15, 7, 12, 9, 0);
    testWrapT<int>(6, -4, 3, -2, 0);
    testWrapT<int>(-19, 9, 13, 11, 0);
}

//------------------------------------------------------------------------------
TEST(Test_functions, Test_wrap_floats) {
    // Try floats.
    testWrapT<float>(-741.325f, 1.4151f, 19.7333f, 9.72113f, 0.0001f);

    float const e = 32.4f;
    int const block = -844;
    float const lb = 1.8f;
    float const ub = 66.7113f;

    float const x = lb + ((ub - lb) * static_cast<float>(block)) + (e - lb);
    testWrapT<float>(x, lb, ub, e, 0.01f);
}

}  // End namespace emerald::util
