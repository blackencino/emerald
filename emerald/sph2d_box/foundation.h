#pragma once

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathColor.h>
#include <OpenEXR/ImathMatrix.h>
#include <OpenEXR/ImathVec.h>

#include <cstdint>
#include <vector>

namespace emerald::sph2d_box {

//------------------------------------------------------------------------------
using C3f = Imath::Color3<float>;
using C4f = Imath::Color4<float>;

using V3f = Imath::Vec3<float>;
using V3i = Imath::Vec3<int32_t>;
using V3ui = Imath::Vec3<uint32_t>;

using V2f = Imath::Vec2<float>;
using V2i = Imath::Vec2<int32_t>;
using V2ui = Imath::Vec2<uint32_t>;

using Box2f = Imath::Box<V2f>;
using Box2i = Imath::Box<V2i>;

using Box3f = Imath::Box<V3f>;
using Box3i = Imath::Box<V3i>;

using M33f = Imath::Matrix33<float>;
using M44f = Imath::Matrix44<float>;

//------------------------------------------------------------------------------
using V3f_array = std::vector<V3f>;
using V3i_array = std::vector<V3i>;
using V3ui_array = std::vector<V3ui>;

using V2f_array = std::vector<V2f>;
using V2i_array = std::vector<V2i>;
using V2ui_array = std::vector<V2ui>;

using C3f_array = std::vector<C3f>;
using C4f_array = std::vector<C4f>;

using Float_array = std::vector<float>;
using Int_array = std::vector<int32_t>;
using Uint_array = std::vector<uint32_t>;

using Uchar_array = std::vector<uint8_t>;

}  // namespace emerald::sph2d_box
