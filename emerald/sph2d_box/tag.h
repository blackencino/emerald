#pragma once

#include <bitset>
#include <cstdint>

namespace emerald::sph2d_box {

using Tag = std::bitset<32>;

constexpr size_t LOW_RES_BIT = 0;
constexpr size_t BOUNDARY_BIT = 1;
constexpr size_t UPRES_BIT = 2;
constexpr size_t SURFACE_BIT = 3;
constexpr size_t NEAR_SURFACE_BIT = 4;
constexpr size_t NEAR_SOLID_TAG = 5;
constexpr size_t NEAR_SOLID_VALLEY_TAG = 6;
constexpr size_t NEAR_SOLID_CORNER_TAG = 7;

}  // namespace emerald::sph2d_box
