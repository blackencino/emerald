#pragma once

#include <emerald/util/foundation.h>

#include <array>
#include <cstdint>
#include <random>

namespace emerald {
namespace util {

//-*****************************************************************************
std::array<uint16_t, 3> HashGridIntoArray(uint32_t const i_seed,
                                          uint32_t const i_x,
                                          uint32_t const i_y,
                                          uint32_t const i_z);

//-*****************************************************************************
inline int64_t HashGrid(uint32_t const i_seed,
                        uint32_t const i_x,
                        uint32_t const i_y,
                        uint32_t const i_z) {
    auto const state = HashGridIntoArray(i_seed, i_x, i_y, i_z);

    // Assemble the 48-bit value x[n] from the
    // three 16-bit values stored in state.
    auto const x = (static_cast<int64_t>(state[2]) << 32) |
                   (static_cast<int64_t>(state[1]) << 16) |
                   (static_cast<int64_t>(state[0]));

    return x;
}

//-*****************************************************************************
class UniformRand {
public:
    explicit UniformRand(double const iMin = 0.0,
                         double const iMax = 1.0,
                         int64_t const iSeed = 54321)
      : m_generator(iSeed)
      , m_distribution(iMin, iMax) {
    }

    void reset(int64_t const iSeed) {
        m_generator.seed(iSeed);
        m_distribution.reset();
    }

    void reset(V3f const& iSeed) {
        auto const* const data = reinterpret_cast<uint32_t const*>(&iSeed.x);
        reset(HashGrid(54321, data[0], data[1], data[2]));
    }

    void reset(int64_t const i_seed0, V3f const& i_seed1) {
        auto const* const data = reinterpret_cast<uint32_t const*>(&i_seed1.x);
        reset(HashGrid(i_seed0, data[0], data[1], data[2]));
    }

    double operator()() {
        return m_distribution(m_generator);
    }

protected:
    std::mt19937_64 m_generator;
    std::uniform_real_distribution<double> m_distribution;
};

//-*****************************************************************************
class GaussRand {
public:
    GaussRand(double const iMean = 0.0,
              double const iDev = 1.0,
              int64_t const iSeed = 54321)
      : m_generator(iSeed)
      , m_distribution(iMean, iDev) {
    }

    void reset(int64_t const iSeed) {
        m_generator.seed(iSeed);
        m_distribution.reset();
    }

    void reset(int32_t const iSeed) {
        m_generator.seed(iSeed);
        m_distribution.reset();
    }

    void reset(V3f const& iSeed) {
        auto const* const data = reinterpret_cast<uint32_t const*>(&iSeed.x);
        reset(HashGrid(54321, data[0], data[1], data[2]));
    }

    void reset(int64_t const i_seed0, V3f const& i_seed1) {
        auto const* const data = reinterpret_cast<uint32_t const*>(&i_seed1.x);
        reset(HashGrid(i_seed0, data[0], data[1], data[2]));
    }

    double operator()() {
        return m_distribution(m_generator);
    }

protected:
    std::mt19937_64 m_generator;
    std::normal_distribution<double> m_distribution;
};

//------------------------------------------------------------------------------
struct Lehmer_rand_gen_64 {
    using result_type = uint64_t;
    static constexpr result_type min() {
        return 0;
    }
    static constexpr result_type max() {
        return std::numeric_limits<uint64_t>::max();
    }

    Lehmer_rand_gen_64() = delete;
    Lehmer_rand_gen_64(uint64_t const seed)
      : state(seed) {
        state *= 0xda942042e4dd58b5;
        state *= 0xda942042e4dd58b5;
    };

    uint64_t operator()() {
        state *= 0xda942042e4dd58b5;
        return state >> 64;
    }

    __uint128_t state;
};

}  // End namespace util
}  // End namespace emerald
