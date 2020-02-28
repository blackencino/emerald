#pragma once

#include <emerald/util/murmur3.h>
#include <emerald/util/storage_traits.h>

#include <array>
#include <string>
#include <vector>

namespace emerald {
namespace util {

//-*****************************************************************************
// FOR COMPUTING THE HASH OF AN ARRAY OF DATA
//-*****************************************************************************

//-*****************************************************************************
using VectorHashKey = std::array<uint64_t, 2>;

std::string FormatHashKey(VectorHashKey const key);

//-*****************************************************************************
template <typename VECTOR>
VectorHashKey ComputeVectorHashKey(VECTOR const& array,
                                   VectorHashKey const hashKey) {
    using value_type = typename VECTOR::value_type;
    using traits_type = storage_traits<value_type>;
    using pod_type = typename traits_type::pod_type;

    if (array.empty()) { return hashKey; }

    return MurmurHash3_x64_128(reinterpret_cast<void const*>(array.data()),
                               sizeof(value_type) * array.size(),
                               sizeof(pod_type),
                               hashKey);
}

//-*****************************************************************************
template <typename VECTOR>
VectorHashKey ComputeVectorHashKey(VECTOR const& array) {
    return ComputeVectorHashKey<VECTOR>(array, {0, 0});
}

//-*****************************************************************************
template <typename T>
VectorHashKey ComputeDataArrayHashKey(
    T const* const data,
    std::size_t const num,
    VectorHashKey const hashKey,
    uint32_t const seed = sizeof(typename storage_traits<T>::pod_type)) {
    if (data && num > 0) {
        void const* const cdata = reinterpret_cast<const void*>(data);

        return MurmurHash3_x64_128(cdata, sizeof(T) * num, seed, hashKey);
    } else {
        return hashKey;
    }
}

//-*****************************************************************************
template <typename T>
VectorHashKey ComputeDataArrayHashKey(
    T const* const data,
    std::size_t const num,
    uint32_t const seed = sizeof(typename storage_traits<T>::pod_type)) {
    return ComputeDataArrayHashKey<T>(data, num, {0, 0}, seed);
}

}  // End namespace util
}  // End namespace emerald
