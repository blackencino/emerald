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

#ifndef _EmldCore_Util_Murmur3_h_
#define _EmldCore_Util_Murmur3_h_

#include "Foundation.h"

#include <cstdint>

namespace EmldCore {
namespace Util {

//-*****************************************************************************
void MurmurHash3_x86_32(const void* i_bytes,
                        std::size_t i_numBytes,
                        uint32_t i_seed,
                        uint32_t* o_out);

//-*****************************************************************************
void MurmurHash3_x86_128(const void* i_bytes,
                         std::size_t i_numBytes,
                         uint32_t i_seed,
                         uint32_t* o_out);

//-*****************************************************************************
void MurmurHash3_x64_128(const void* i_bytes,
                         std::size_t i_numBytes,
                         uint32_t i_seed,
                         uint64_t* o_out);

//-*****************************************************************************
//-*****************************************************************************
// Initializes the out to zero before going.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
inline void MurmurHash3_x86_32_InitZero(const void* i_bytes,
                                        std::size_t i_numBytes,
                                        uint32_t i_seed,
                                        uint32_t* o_out) {
    o_out[0] = 0;
    MurmurHash3_x86_32(i_bytes, i_numBytes, i_seed, o_out);
}

//-*****************************************************************************
inline void MurmurHash3_x86_128_InitZero(const void* i_bytes,
                                         std::size_t i_numBytes,
                                         uint32_t i_seed,
                                         uint32_t* o_out) {
    o_out[0] = 0;
    o_out[1] = 0;
    o_out[2] = 0;
    o_out[3] = 0;
    MurmurHash3_x86_128(i_bytes, i_numBytes, i_seed, o_out);
}

//-*****************************************************************************
inline void MurmurHash3_x64_128_InitZero(const void* i_bytes,
                                         std::size_t i_numBytes,
                                         uint32_t i_seed,
                                         uint64_t* o_out) {
    o_out[0] = 0;
    o_out[1] = 0;
    MurmurHash3_x64_128(i_bytes, i_numBytes, i_seed, o_out);
}

}  // End namespace Util
}  // End namespace EmldCore

#endif
