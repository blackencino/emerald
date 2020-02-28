#pragma once

#include <emerald/noise/foundation.h>

namespace emerald {
namespace noise {

//-*****************************************************************************
float CellNoise(unsigned long int iSeeds[], int numSeeds);

//-*****************************************************************************
template <class T>
float CellNoise(const Imath::Vec3<T>& iPoint) {
    unsigned long int seeds[4];
    seeds[0] = 0;
    seeds[1] = 0L + (long int)std::floor(iPoint[0]);
    seeds[2] = 0L + (long int)std::floor(iPoint[1]);
    seeds[3] = 0L + (long int)std::floor(iPoint[2]);
    return CellNoise(seeds, 4);
}

//-*****************************************************************************
template <class T>
Imath::V2f CellNoise2(const Imath::Vec3<T>& iPoint) {
    unsigned long int seeds[4];
    seeds[0] = 0;
    seeds[1] = 0L + (long int)std::floor(iPoint[0]);
    seeds[2] = 0L + (long int)std::floor(iPoint[1]);
    seeds[3] = 0L + (long int)std::floor(iPoint[2]);
    Imath::V2f ret;
    ret.x = CellNoise(seeds, 4);
    seeds[0] = 1;
    ret.y = CellNoise(seeds, 4);
    return ret;
}

//-*****************************************************************************
template <class T>
Imath::V3f CellNoise3(const Imath::Vec3<T>& iPoint) {
    unsigned long int seeds[4];
    seeds[0] = 0;
    seeds[1] = 0L + (long int)std::floor(iPoint[0]);
    seeds[2] = 0L + (long int)std::floor(iPoint[1]);
    seeds[3] = 0L + (long int)std::floor(iPoint[2]);
    Imath::V3f ret;
    ret.x = CellNoise(seeds, 4);
    seeds[0] = 1;
    ret.y = CellNoise(seeds, 4);
    seeds[0] = 2;
    ret.z = CellNoise(seeds, 4);
    return ret;
}

}  // End namespace noise
}  // End namespace emerald
