#pragma once

#include <emerald/util/foundation.h>

#include <array>
#include <cstddef>

namespace emerald {
namespace util {

//-*****************************************************************************
// A traits class is a mechanism that we assume will be overridden with
// template specializations.
// When we make arrays of data for simulations, we often want to alias
// the same memory for different interpreted types
// So, for example, we might use the same storage for an array of V3f's
// as we do for an array of V3i's, and so on.
// Additionally, we sometimes want to be able to perform really generic
// low-level functions on arrays of data, such as computing a hash-key,
// as alembic does.
// So - we have this traits class which provides a typedef for the value_type,
// the storage_type, the number of bytes, and the POD_TYPE of the primary
// value_type, where POD_TYPE has the usual meaning.
//-*****************************************************************************

template <typename T>
struct storage_traits {
    using value_type = T;
    using storage_type = T;
    using pod_type = T;
};

//-*****************************************************************************
// Common storage types, expressed via the largest pod.
template <std::size_t N>
using storage_bytes = std::array<std::byte, N>;

using storage_bytes_1 = std::array<std::byte, 1>;
using storage_bytes_2 = std::array<std::byte, 2>;
using storage_bytes_3 = std::array<std::byte, 3>;
using storage_bytes_4 = std::array<std::byte, 4>;
using storage_bytes_6 = std::array<std::byte, 6>;
using storage_bytes_8 = std::array<std::byte, 8>;
using storage_bytes_12 = std::array<std::byte, 12>;
using storage_bytes_16 = std::array<std::byte, 16>;
using storage_bytes_24 = std::array<std::byte, 24>;
using storage_bytes_32 = std::array<std::byte, 32>;
using storage_bytes_36 = std::array<std::byte, 36>;
using storage_bytes_40 = std::array<std::byte, 40>;
using storage_bytes_52 = std::array<std::byte, 52>;
using storage_bytes_56 = std::array<std::byte, 56>;
using storage_bytes_64 = std::array<std::byte, 64>;
using storage_bytes_72 = std::array<std::byte, 72>;
using storage_bytes_80 = std::array<std::byte, 80>;
using storage_bytes_96 = std::array<std::byte, 96>;
using storage_bytes_128 = std::array<std::byte, 128>;
using storage_bytes_256 = std::array<std::byte, 256>;

//-*****************************************************************************
// Specializations
#define EMLD_SPECIALIZE_STORAGE_TRAITS(VAL, STORE, POD) \
    template <>                                         \
    struct storage_traits<VAL> {                        \
        static_assert(sizeof(VAL) == sizeof(STORE),     \
                      "size of storage type must be "   \
                      "same as size of value type. "    \
                      "val: " #VAL "storage: " #STORE); \
        using value_type = VAL;                         \
        using storage_type = STORE;                     \
        using pod_type = POD;                           \
    }

//-*****************************************************************************
EMLD_SPECIALIZE_STORAGE_TRAITS(uint8_t, storage_bytes<1>, uint8_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(int8_t, storage_bytes<1>, int8_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(uint16_t, storage_bytes<2>, uint16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(int16_t, storage_bytes<2>, int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(uint32_t, storage_bytes<4>, uint32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(int32_t, storage_bytes<4>, int32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(uint64_t, storage_bytes<8>, uint64_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(int64_t, storage_bytes<8>, int64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(float16_t, storage_bytes<2>, float16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(float32_t, storage_bytes<4>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(float64_t, storage_bytes<8>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(V2s, storage_bytes<4>, int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(V2i, storage_bytes<8>, int32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(V2f, storage_bytes<8>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(V2d, storage_bytes<16>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(Imath::Vec2<float16_t>,
                               storage_bytes<4>,
                               int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Imath::Vec2<uint32_t>,
                               storage_bytes<8>,
                               uint32_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(V3s, storage_bytes<6>, int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(V3i, storage_bytes<12>, int32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(V3f, storage_bytes<12>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(V3d, storage_bytes<24>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(Imath::Vec3<float16_t>,
                               storage_bytes<6>,
                               int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Imath::Vec3<uint32_t>,
                               storage_bytes<12>,
                               uint32_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(Box2s, storage_bytes<8>, int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Box2i, storage_bytes<16>, int32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Box2f, storage_bytes<16>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Box2d, storage_bytes<32>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(Box3s, storage_bytes<12>, int16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Box3i, storage_bytes<24>, int32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Box3f, storage_bytes<24>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Box3d, storage_bytes<48>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(M33f, storage_bytes<36>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(M33d, storage_bytes<72>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(M44f, storage_bytes<64>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(M44d, storage_bytes<128>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(Quatf, storage_bytes<16>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(Quatd, storage_bytes<32>, float64_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(C3h, storage_bytes<6>, float16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(C3f, storage_bytes<12>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(C3c, storage_bytes<3>, uint8_t);

EMLD_SPECIALIZE_STORAGE_TRAITS(C4h, storage_bytes<8>, float16_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(C4f, storage_bytes<16>, float32_t);
EMLD_SPECIALIZE_STORAGE_TRAITS(C4c, storage_bytes<4>, uint8_t);

}  // End namespace util
}  // End namespace emerald
