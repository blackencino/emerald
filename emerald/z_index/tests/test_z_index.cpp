#include <emerald/z_index/z_index.h>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace emerald::z_index {

static const Z_index_2 g_z_index_2;
static const Z_index_3 g_z_index_3;

TEST(Z_index_test, Z_index_2_single_axis_index_from_lut) {
    constexpr uint32_t value_1 = 21430205;
    constexpr auto expected_index_1 = Single_axis_index<2, 31>::encode(value_1);
    auto const index_1 = g_z_index_2.single_axis_index_from_lut(value_1);
    EXPECT_EQ(expected_index_1, index_1);

    constexpr uint32_t value_2 = 0b111ul;
    constexpr auto expected_index_2 = 0b10101ul;
    constexpr auto expected_index_2b =
        Single_axis_index<2, 31>::encode(value_2);
    auto const index_2 = g_z_index_2.single_axis_index_from_lut(value_2);
    EXPECT_EQ(expected_index_2, expected_index_2b);
    EXPECT_EQ(expected_index_2, index_2);

    constexpr uint32_t value_3 = 0b1111'1111'1111'1111ul;
    constexpr auto expected_index_3 =
        0b0101'0101'0101'0101'0101'0101'0101'0101ull;
    constexpr auto expected_index_3b =
        Single_axis_index<2, 31>::encode(value_3);
    auto const index_3 = g_z_index_2.single_axis_index_from_lut(value_3);
    EXPECT_EQ(expected_index_3, expected_index_3b);
    EXPECT_EQ(expected_index_3, index_3);

    constexpr uint32_t value_4 = 0b1111'1111'0000'0000ul;
    constexpr auto expected_index_4 =
        0b0101'0101'0101'0101'0000'0000'0000'0000ull;
    constexpr auto expected_index_4b =
        Single_axis_index<2, 31>::encode(value_4);
    auto const index_4 = g_z_index_2.single_axis_index_from_lut(value_4);
    EXPECT_EQ(expected_index_4, expected_index_4b);
    EXPECT_EQ(expected_index_4, index_4);
}

TEST(Z_index_test, Z_index_2_encode) {
    constexpr uint32_t x_1 = 0b111ul;
    constexpr uint32_t y_1 = 0b111ul;
    constexpr auto expected_index_1 = 0b0011'1111ull;
    auto const index_1 = z_index(x_1, y_1);
    EXPECT_EQ(expected_index_1, index_1);

    constexpr uint32_t x_2 = 0b1111'1111'1111'1111ul;
    constexpr uint32_t y_2 = 0b1111'1111'1111'1111ul;
    constexpr uint64_t expected_index_2 =
        0b1111'1111'1111'1111'1111'1111'1111'1111ull;
    auto const index_2 = z_index(x_2, y_2);
    EXPECT_EQ(expected_index_2, index_2);

    constexpr uint32_t x_3 = 0b0000'0000'0000'0000ul;
    constexpr uint32_t y_3 = 0b1111'1111'1111'1111ul;
    constexpr uint64_t expected_index_3 =
        0b1010'1010'1010'1010'1010'1010'1010'1010ull;
    auto const index_3 = z_index(x_3, y_3);
    EXPECT_EQ(expected_index_3, index_3);

    constexpr uint32_t x_4 = 0xff'ff'ff'fful;
    constexpr uint32_t y_4 = 0xff'ff'ff'fful;
    constexpr uint64_t expected_index_4 = 0xffff'ffff'ffff'ffffull;
    auto const index_4 = z_index(x_4, y_4);
    EXPECT_EQ(expected_index_4, index_4);
}

TEST(Z_index_test, Z_index_3_single_axis_index_from_lut) {
    constexpr uint32_t value_1 = 2089375;
    constexpr auto expected_index_1 = Single_axis_index<3, 21>::encode(value_1);
    auto const index_1 = g_z_index_3.single_axis_index_from_lut(value_1);
    EXPECT_EQ(expected_index_1, index_1);

    constexpr uint32_t value_2 = 0b111ul;
    constexpr auto expected_index_2 = 0b1001001ul;
    constexpr auto expected_index_2b =
        Single_axis_index<3, 21>::encode(value_2);
    auto const index_2 = g_z_index_3.single_axis_index_from_lut(value_2);
    EXPECT_EQ(expected_index_2, expected_index_2b);
    EXPECT_EQ(expected_index_2, index_2);
}

TEST(Z_index_test, Z_index_3_encode) {
    constexpr uint32_t x_1 = 0b111ul;
    constexpr uint32_t y_1 = 0b111ul;
    constexpr uint32_t z_1 = 0b111ul;
    constexpr auto expected_index_1 = 0b1'1111'1111ull;
    auto const index_1 = z_index(x_1, y_1, z_1);
    EXPECT_EQ(expected_index_1, index_1);

    constexpr uint32_t x_2 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint32_t y_2 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint32_t z_2 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint64_t expected_index_2 =
        0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111ull;
    auto const index_2 = z_index(x_2, y_2, z_2);
    EXPECT_EQ(expected_index_2, index_2);

    constexpr uint32_t x_3 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint32_t y_3 = 0b0'0000'0000'0000'0000'0000ul;
    constexpr uint32_t z_3 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint64_t expected_index_3 =
        0b0101'1011'0110'1101'1011'0110'1101'1011'0110'1101'1011'0110'1101'1011'0110'1101ull;
    auto const index_3 = z_index(x_3, y_3, z_3);
    EXPECT_EQ(expected_index_3, index_3);

    constexpr uint32_t x_4 = 0b111'1111'1111'1111'1111'1111ul;
    constexpr uint32_t y_4 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint32_t z_4 = 0b1'1111'1111'1111'1111'1111ul;
    constexpr uint64_t expected_index_4 =
        0b0111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111'1111ull;
    auto const index_4 = z_index(x_4, y_4, z_4);
    EXPECT_EQ(expected_index_4, index_4);
}

}  // namespace emerald::z_index
