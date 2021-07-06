#pragma once

#pragma warning(push)
#pragma warning( disable : 4244 4100 4456 )

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathColor.h>
#include <OpenEXR/ImathMatrix.h>
#include <OpenEXR/ImathVec.h>

#include <cstdint>
#include <type_traits>
#include <vector>

namespace emerald::sph_common {

//------------------------------------------------------------------------------
using C3f = Imath::Color3<float>;
using C4f = Imath::Color4<float>;
using C4uc = Imath::Color4<uint8_t>;
using C4d = Imath::Color4<double>;

using V3f = Imath::Vec3<float>;
using V3i = Imath::Vec3<int32_t>;
using V3ui = Imath::Vec3<uint32_t>;
using V3d = Imath::Vec3<double>;

using V2f = Imath::Vec2<float>;
using V2i = Imath::Vec2<int32_t>;
using V2ui = Imath::Vec2<uint32_t>;
using V2d = Imath::Vec2<double>;

using Box2f = Imath::Box<V2f>;
using Box2i = Imath::Box<V2i>;

using Box3f = Imath::Box<V3f>;
using Box3i = Imath::Box<V3i>;

using M33f = Imath::Matrix33<float>;
using M44f = Imath::Matrix44<float>;
using M33d = Imath::Matrix33<double>;
using M44d = Imath::Matrix44<double>;

template <typename T, typename S>
inline std::enable_if_t<std::is_arithmetic_v<T> && std::is_arithmetic_v<S>,
                        Imath::Vec2<T>>
vec_cast(Imath::Vec2<S> const& v) {
    return Imath::Vec2<T>{static_cast<T>(v[0]), static_cast<T>(v[1])};
}

template <typename T, typename S>
inline std::enable_if_t<std::is_arithmetic_v<T> && std::is_arithmetic_v<S>,
                        Imath::Vec3<T>>
vec_cast(Imath::Vec3<S> const& v) {
    return Imath::Vec3<T>{
      static_cast<T>(v[0]), static_cast<T>(v[1]), static_cast<T>(v[2])};
}

}  // namespace emerald::sph_common

#pragma warning(pop)
