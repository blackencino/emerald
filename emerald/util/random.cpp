#include <emerald/util/random.h>

#include <array>

namespace emerald {
namespace util {

//-*****************************************************************************
// Tiny Encryption Algorithm for turning hashes into good random numbers.
// Works in-place.
static std::array<uint32_t, 2> Tea8(std::array<uint32_t, 2> v) {
    static constexpr std::array<uint32_t, 4> k = {
        0xa341316c, 0xc8013ea4, 0xad90777d, 0x7395761e};

    static constexpr uint32_t delta = 0x9e3779b9;

    uint32_t sum = 0;

    for (int i = 0; i < 8; ++i) {
        sum += delta;
        v[0] += ((v[1] << 4) + k[0]) ^ (v[1] + sum) ^ ((v[1] >> 5) + k[1]);
        v[1] += ((v[0] << 4) + k[2]) ^ (v[0] + sum) ^ ((v[0] >> 5) + k[3]);
    }

    return v;
}

//-*****************************************************************************
std::array<uint16_t, 3> HashGridIntoArray(uint32_t const seed,
                                          uint32_t const x,
                                          uint32_t const y,
                                          uint32_t const z) {
    union {
        std::array<uint32_t, 2> v;
        std::array<uint16_t, 3> x;
    } state;

    state.v[0] = seed;
    state.v[1] = x;
    state.v = Tea8(state.v);

    state.v[0] += y;
    state.v[1] += z;
    state.v = Tea8(state.v);

    return state.x;
}

}  // End namespace util
}  // End namespace emerald
