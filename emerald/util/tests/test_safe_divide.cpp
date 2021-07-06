#include <emerald/util/foundation.h>
#include <emerald/util/safe_divide.h>

#include <gtest/gtest.h>

#include <cstdint>

namespace emerald::util {

#define EXPECT_V2F_EQ(expected, expression)       \
    {                                             \
        auto const value = (expression);          \
        EXPECT_FLOAT_EQ((expected)[0], value[0]); \
        EXPECT_FLOAT_EQ((expected)[1], value[1]); \
    }

TEST(Safe_divide_test, test_integers) {
    int const numer1 = 0;
    int const denom1 = 0;
    EXPECT_FALSE(is_safe_divide(numer1, denom1));
    EXPECT_FALSE(safe_divide(numer1, denom1).has_value());

    int const numer2 = 7;
    int const denom2 = 0;
    EXPECT_FALSE(is_safe_divide(numer2, denom2));
    EXPECT_FALSE(safe_divide(numer2, denom2).has_value());

    int const numer3 = 7;
    int const denom3 = -10000000;
    EXPECT_TRUE(is_safe_divide(numer3, denom3));
    ASSERT_TRUE(safe_divide(numer3, denom3).has_value());
    EXPECT_EQ(0, safe_divide(numer3, denom3).value());

    int const numer4 = -16;
    int const denom4 = -4;
    EXPECT_TRUE(is_safe_divide(numer4, denom4));
    ASSERT_TRUE(safe_divide(numer4, denom4).has_value());
    EXPECT_EQ(4, safe_divide(numer4, denom4).value());
}

TEST(Safe_divide_test, test_floats) {
    float const numer1 = 0.0f;
    float const denom1 = 0.0f;
    EXPECT_FALSE(is_safe_divide(numer1, denom1));
    EXPECT_FALSE(safe_divide(numer1, denom1).has_value());

    float const numer2 = 1.0e-37f;
    float const denom2 = 1.0e-37f;
    EXPECT_TRUE(is_safe_divide(numer2, denom2));
    ASSERT_TRUE(safe_divide(numer2, denom2).has_value());
    EXPECT_EQ(1.0f, safe_divide(numer2, denom2).value());

    float const numer3 = 1.0e37f;
    float const denom3 = 1.0e-2f;
    EXPECT_FALSE(is_safe_divide(numer3, denom3));

    float const numer4 = 1.0e38f;
    float const denom4 = 1.0e-1f;
    EXPECT_FALSE(is_safe_divide(numer4, denom4));

    float const numer5 = -16.0f;
    float const denom5 = -4.0f;
    EXPECT_TRUE(is_safe_divide(numer5, denom5));
    ASSERT_TRUE(safe_divide(numer5, denom5).has_value());
    EXPECT_FLOAT_EQ(4.0f, safe_divide(numer5, denom5).value());

    float const numer6 = 1.0f;
    float const denom6 = -10.0f;
    EXPECT_TRUE(is_safe_divide(numer6, denom6));
    ASSERT_TRUE(safe_divide(numer6, denom6).has_value());
    EXPECT_FLOAT_EQ(-0.1f, safe_divide(numer6, denom6).value());
}

TEST(Safe_divide_test, test_doubles) {
    double const numer1 = 0.0;
    double const denom1 = 0.0;
    EXPECT_FALSE(is_safe_divide(numer1, denom1));
    EXPECT_FALSE(safe_divide(numer1, denom1).has_value());

    double const numer2 = 2.0e-307;
    double const denom2 = 2.0e-307;
    EXPECT_TRUE(is_safe_divide(numer2, denom2));
    ASSERT_TRUE(safe_divide(numer2, denom2).has_value());
    EXPECT_EQ(1.0, safe_divide(numer2, denom2).value());

    double const numer3 = 2.0e307;
    double const denom3 = 1.0e-2;
    EXPECT_FALSE(is_safe_divide(numer3, denom3));

    double const numer4 = 1.0e308;
    double const denom4 = 1.0e-1;
    EXPECT_FALSE(is_safe_divide(numer4, denom4));

    double const numer5 = -16.0;
    double const denom5 = -4.0;
    EXPECT_TRUE(is_safe_divide(numer5, denom5));
    ASSERT_TRUE(safe_divide(numer5, denom5).has_value());
    EXPECT_DOUBLE_EQ(4.0, safe_divide(numer5, denom5).value());

    double const numer6 = 1.0;
    double const denom6 = -10.0;
    EXPECT_TRUE(is_safe_divide(numer6, denom6));
    ASSERT_TRUE(safe_divide(numer6, denom6).has_value());
    EXPECT_DOUBLE_EQ(-0.1, safe_divide(numer6, denom6).value());
}

TEST(Safe_divide_test, test_V2i) {
    V2i const numer1{0, 0};
    V2i const numer1a{1, 0};
    V2i const numer1b{0, 1};
    V2i const numer1c{7, -7};
    int const denom1 = 0;
    int const denom2 = 7;
    EXPECT_FALSE(is_safe_divide(numer1, denom1));
    EXPECT_FALSE(safe_divide(numer1, denom1).has_value());

    EXPECT_FALSE(is_safe_divide(numer1a, denom1));
    EXPECT_FALSE(safe_divide(numer1a, denom1).has_value());

    EXPECT_FALSE(is_safe_divide(numer1b, denom1));
    EXPECT_FALSE(safe_divide(numer1b, denom1).has_value());

    EXPECT_FALSE(is_safe_divide(numer1c, denom1));
    EXPECT_FALSE(safe_divide(numer1c, denom1).has_value());

    EXPECT_TRUE(is_safe_divide(numer1, denom2));
    ASSERT_TRUE(safe_divide(numer1, denom2).has_value());
    V2i const expected1{0, 0};
    EXPECT_EQ(expected1, safe_divide(numer1, denom2).value());

    EXPECT_TRUE(is_safe_divide(numer1a, denom2));
    ASSERT_TRUE(safe_divide(numer1a, denom2).has_value());
    V2i const expected1a{0, 0};
    EXPECT_EQ(expected1a, safe_divide(numer1a, denom2).value());

    EXPECT_TRUE(is_safe_divide(numer1b, denom2));
    ASSERT_TRUE(safe_divide(numer1b, denom2).has_value());
    V2i const expected1b{0, 0};
    EXPECT_EQ(expected1b, safe_divide(numer1b, denom2).value());

    EXPECT_TRUE(is_safe_divide(numer1c, denom2));
    ASSERT_TRUE(safe_divide(numer1c, denom2).has_value());
    V2i const expected1c{1, -1};
    EXPECT_EQ(expected1c, safe_divide(numer1c, denom2).value());
}

TEST(Safe_divide_test, test_V2f) {
    V2f const numer1{0.0f, 0.0f};
    float const denom1 = 0.0f;
    EXPECT_FALSE(is_safe_divide(numer1, denom1));
    EXPECT_FALSE(safe_divide(numer1, denom1).has_value());

    V2f const numer2{1.0e-37f, 1.0e-37f};
    float const denom2 = 1.0e-37f;
    EXPECT_TRUE(is_safe_divide(numer2, denom2));
    ASSERT_TRUE(safe_divide(numer2, denom2).has_value());
    V2f const expected2{1.0f, 1.0f};
    EXPECT_EQ(expected2, safe_divide(numer2, denom2).value());

    V2f const numer3{1.0e37f, 1.0e37f};
    float const denom3 = 1.0e-2f;
    EXPECT_FALSE(is_safe_divide(numer3, denom3));

    V2f const numer4{1.0e38f, 1.0e38f};
    float const denom4 = 1.0e-1f;
    EXPECT_FALSE(is_safe_divide(numer4, denom4));

    V2f const numer5{-16.0f, 1.0f};
    float const denom5 = -4.0f;
    EXPECT_TRUE(is_safe_divide(numer5, denom5));
    ASSERT_TRUE(safe_divide(numer5, denom5).has_value());
    V2f const expected5{4.0f, -0.25f};
    EXPECT_V2F_EQ(expected5, safe_divide(numer5, denom5).value());

    V2f const numer6{1.0f, -0.1f};
    float const denom6 = -10.0f;
    EXPECT_TRUE(is_safe_divide(numer6, denom6));
    ASSERT_TRUE(safe_divide(numer6, denom6).has_value());
    V2f const expected6{-0.1f, 0.01f};
    EXPECT_V2F_EQ(expected6, safe_divide(numer6, denom6).value());
}

TEST(Safe_divide_test, test_V3f) {
    V3f const numer1{0.0f, 0.0f, 0.0f};
    float const denom1 = 0.0f;
    EXPECT_FALSE(is_safe_divide(numer1, denom1));
    EXPECT_FALSE(safe_divide(numer1, denom1).has_value());

    V3f const numer2{1.0e-37f, 1.0e-37f, 0.0f};
    float const denom2 = 1.0e-37f;
    EXPECT_TRUE(is_safe_divide(numer2, denom2));
    ASSERT_TRUE(safe_divide(numer2, denom2).has_value());
    V3f const expected2{1.0f, 1.0f, 0.0f};
    EXPECT_EQ(expected2, safe_divide(numer2, denom2).value());

    V3f const numer3{1.0e37f, 1.0e37f, 0.0f};
    float const denom3 = 1.0e-2f;
    EXPECT_FALSE(is_safe_divide(numer3, denom3));

    V3f const numer4{1.0e38f, 1.0e38f, 0.0f};
    float const denom4 = 1.0e-1f;
    EXPECT_FALSE(is_safe_divide(numer4, denom4));

    V3f const numer5{-16.0f, 1.0f, -0.5f};
    float const denom5 = -4.0f;
    EXPECT_TRUE(is_safe_divide(numer5, denom5));
    ASSERT_TRUE(safe_divide(numer5, denom5).has_value());
    V3f const expected5{4.0f, -0.25f, 0.125f};
    EXPECT_V2F_EQ(expected5, safe_divide(numer5, denom5).value());

    V3f const numer6{1.0f, -0.1f, 0.01f};
    float const denom6 = -10.0f;
    EXPECT_TRUE(is_safe_divide(numer6, denom6));
    ASSERT_TRUE(safe_divide(numer6, denom6).has_value());
    V3f const expected6{-0.1f, 0.01f, -0.001f};
    EXPECT_V2F_EQ(expected6, safe_divide(numer6, denom6).value());
}

}  // namespace emerald::util