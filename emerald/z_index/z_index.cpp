#include <emerald/z_index/z_index.h>

namespace emerald::z_index {

static const Z_index_2 g_z_index_2;
static const Z_index_3 g_z_index_3;

uint64_t z_index(uint32_t const x, uint32_t const y) {
    return g_z_index_2.encode(x, y);
}

uint64_t z_index(uint32_t const x, uint32_t const y, uint32_t const z) {
    return g_z_index_3.encode(x, y, z);
}

//------------------------------------------------------------------------------
Z_index_2::Z_index_2() {
    // The 2d z-index uses a 16-bit LUT
    m_single_axis_lut.resize(65536);

    for (size_t i = 0; i < 65536; ++i) {
        m_single_axis_lut[i] = static_cast<uint32_t>(
          Single_axis_index<2, 15>::encode(static_cast<int>(i)));
    }
}

uint64_t Z_index_2::single_axis_index_from_lut(uint32_t const i) const {
    // take the low-16 bits
    constexpr uint64_t LOW_16_BIT_MASK = (0x1ull << 16) - 1;
    static_assert(LOW_16_BIT_MASK == 0b1111'1111'1111'1111ull, "low 16 bits");

    constexpr uint64_t HIGH_16_BIT_MASK = LOW_16_BIT_MASK << 16;
    static_assert(
      HIGH_16_BIT_MASK == 0b1111'1111'1111'1111'0000'0000'0000'0000ull,
      "high 16 bits");

    // get the low mask...
    return static_cast<uint64_t>(m_single_axis_lut[i & LOW_16_BIT_MASK]) |

           // get the high mask...
           (static_cast<uint64_t>(
              m_single_axis_lut[(i & HIGH_16_BIT_MASK) >> 16])
            << 32);
}

uint64_t Z_index_2::encode(uint32_t const x, uint32_t const y) const {
    return single_axis_index_from_lut(x) | (single_axis_index_from_lut(y) << 1);
}

//------------------------------------------------------------------------------
Z_index_3::Z_index_3() {
    // The 3d z-index uses a 20-bit LUT
    m_single_axis_lut.resize(2097152);

    for (size_t i = 0; i < 2097152; ++i) {
        m_single_axis_lut[i] = Single_axis_index<3, 20>::encode(i);
    }
}

uint64_t Z_index_3::single_axis_index_from_lut(uint32_t const i) const {
    // take the low-21 bits
    constexpr uint64_t LOW_21_BIT_MASK = (0x1ull << 21) - 1;
    static_assert(LOW_21_BIT_MASK == 0b1'1111'1111'1111'1111'1111ull,
                  "low 21 bits");
    return m_single_axis_lut[i & LOW_21_BIT_MASK];
}

uint64_t Z_index_3::encode(uint32_t const x,
                           uint32_t const y,
                           uint32_t const z) const {
    return single_axis_index_from_lut(x) |
           (single_axis_index_from_lut(y) << 1) |
           (single_axis_index_from_lut(z) << 2);
}

}  // namespace emerald::z_index
