//-*****************************************************************************
// MurmurHash3 was written by Austin Appleby, and is placed in the public
// domain. The author hereby disclaims copyright to this source code.
//
// Note - The x86 and x64 versions do _not_ produce the same results, as the
// algorithms are optimized for their respective platforms. You can still
// compile and run any of them on any platform, but your performance with the
// non-native version will be less than optimal.
//-*****************************************************************************

//-*****************************************************************************
// CJH NOTE - I've changed the functions to use an already existing hash
// as a starting point, so that murmur hashes can be accumulated.
//-*****************************************************************************

#pragma once

#include <array>
#include <cstdint>

namespace emerald {
namespace util {

//-*****************************************************************************
uint32_t MurmurHash3_x86_32(void const* const i_bytes,
                            std::size_t const i_numBytes,
                            uint32_t const i_seed,
                            uint32_t const init = 0);

//-*****************************************************************************
std::array<uint32_t, 4> MurmurHash3_x86_128(
    void const* const i_bytes,
    std::size_t const i_numBytes,
    uint32_t const i_seed,
    std::array<uint32_t, 4> const init = {0, 0, 0, 0});

//-*****************************************************************************
std::array<uint64_t, 2> MurmurHash3_x64_128(
    void const* const i_bytes,
    std::size_t const i_numBytes,
    uint32_t const i_seed,
    std::array<uint64_t, 2> const init = {0, 0});

}  // End namespace util
}  // End namespace emerald
