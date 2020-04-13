#pragma once

#include <cstdint>
#include <vector>

namespace emerald::z_index {

uint64_t z_index(uint32_t const x, uint32_t const y);
uint64_t z_index(uint32_t const x, uint32_t const y, uint32_t const z);

template <uint8_t DIM, uint8_t BIT>
struct Single_axis_index {
    static constexpr uint64_t encode(uint64_t const i) {
        return ((i & (0x1ull << BIT)) << (BIT * (DIM-1))) |
               Single_axis_index<DIM, BIT - 1>::encode(i);
    }
};

template <uint8_t DIM>
struct Single_axis_index<DIM, 0> {
    static constexpr uint64_t encode(uint64_t const i) {
        return i & 0x1ull;
    }
};

class Z_index_2 {
public:
    Z_index_2();
    uint64_t single_axis_index_from_lut(uint32_t const i) const;
    uint64_t encode(uint32_t const x, uint32_t const y) const;

private:
    std::vector<uint32_t> m_single_axis_lut;
};

class Z_index_3 {
public:
    Z_index_3();
    uint64_t single_axis_index_from_lut(uint32_t const i) const;
    uint64_t encode(uint32_t const x, uint32_t const y, uint32_t const z) const;

private:
    std::vector<uint64_t> m_single_axis_lut;
};

}  // namespace emerald::z_index
