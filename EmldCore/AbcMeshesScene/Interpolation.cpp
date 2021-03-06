//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#include "Interpolation.h"

#include <Alembic/AbcGeom/All.h>
#include <OpenEXR/ImathMatrixAlgo.h>

#include <utility>

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
template <typename IPROP>
typename IPROP::value_type InterpolateScalar(IPROP& i_prop, chrono_t i_time) {
    if (i_prop.isConstant()) { return i_prop.getValue(); }

    // Get samples
    size_t numSamps = i_prop.getNumSamples();
    AbcG::TimeSamplingPtr tsmp = i_prop.getTimeSampling();
    std::pair<index_t, chrono_t> idx0 = tsmp->getFloorIndex(i_time, numSamps);
    std::pair<index_t, chrono_t> idx1 = tsmp->getCeilIndex(i_time, numSamps);
    double interp =
      (i_time - idx0.second) / std::max(idx1.second - idx0.second, 0.00001);

    typename IPROP::value_type f0;
    i_prop.get(f0, idx0.first);

    typename IPROP::value_type f1;
    i_prop.get(f1, idx1.first);

    double d0 = static_cast<double>(f0);
    double d1 = static_cast<double>(f1);

    double d = Imath::lerp(d0, d1, interp);

    return static_cast<typename IPROP::value_type>(d);
}

//-*****************************************************************************
template <typename IPROP>
typename IPROP::value_type InterpolateVector(IPROP& i_prop, chrono_t i_time) {
    if (i_prop.isConstant()) { return i_prop.getValue(); }

    // Get samples
    size_t numSamps = i_prop.getNumSamples();
    AbcG::TimeSamplingPtr tsmp = i_prop.getTimeSampling();
    std::pair<index_t, chrono_t> idx0 = tsmp->getFloorIndex(i_time, numSamps);
    std::pair<index_t, chrono_t> idx1 = tsmp->getCeilIndex(i_time, numSamps);
    double interp =
      (i_time - idx0.second) / std::max(idx1.second - idx0.second, 0.00001);

    typename IPROP::value_type f0;
    i_prop.get(f0, idx0.first);

    typename IPROP::value_type f1;
    i_prop.get(f1, idx1.first);

    V3d d0(f0);
    V3d d1(f1);

    V3d d = Imath::lerp(d0, d1, interp);

    typename IPROP::value_type ret(d);

    return ret;
}

//-*****************************************************************************
template <typename IPROP>
typename IPROP::value_type InterpolateOther(IPROP& i_prop, chrono_t i_time) {
    if (i_prop.isConstant()) { return i_prop.getValue(); }

    // Get samples
    size_t numSamps = i_prop.getNumSamples();
    AbcG::TimeSamplingPtr tsmp = i_prop.getTimeSampling();
    std::pair<index_t, chrono_t> idx0 = tsmp->getFloorIndex(i_time, numSamps);

    return i_prop.getValue(idx0.first);
}

//-*****************************************************************************
bool Interpolate(AbcG::IBoolProperty& i_prop, chrono_t i_time) {
    return InterpolateOther<AbcG::IBoolProperty>(i_prop, i_time);
}

//-*****************************************************************************
int32_t Interpolate(AbcG::IInt32Property& i_prop, chrono_t i_time) {
    return InterpolateScalar<AbcG::IInt32Property>(i_prop, i_time);
}

//-*****************************************************************************
int64_t Interpolate(AbcG::IInt64Property& i_prop, chrono_t i_time) {
    return InterpolateScalar<AbcG::IInt64Property>(i_prop, i_time);
}

//-*****************************************************************************
float Interpolate(AbcG::IFloatProperty& i_prop, chrono_t i_time) {
    return InterpolateScalar<AbcG::IFloatProperty>(i_prop, i_time);
}

//-*****************************************************************************
double Interpolate(AbcG::IDoubleProperty& i_prop, chrono_t i_time) {
    return InterpolateScalar<AbcG::IDoubleProperty>(i_prop, i_time);
}

//-*****************************************************************************
V3f Interpolate(AbcG::IV3fProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IV3fProperty>(i_prop, i_time);
}

//-*****************************************************************************
V3d Interpolate(AbcG::IV3dProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IV3dProperty>(i_prop, i_time);
}

//-*****************************************************************************
V3f Interpolate(AbcG::IP3fProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IP3fProperty>(i_prop, i_time);
}

//-*****************************************************************************
V3d Interpolate(AbcG::IP3dProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IP3dProperty>(i_prop, i_time);
}

//-*****************************************************************************
N3f Interpolate(AbcG::IN3fProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IN3fProperty>(i_prop, i_time);
}

//-*****************************************************************************
N3d Interpolate(AbcG::IN3dProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IN3dProperty>(i_prop, i_time);
}

//-*****************************************************************************
C3f Interpolate(AbcG::IC3fProperty& i_prop, chrono_t i_time) {
    return InterpolateVector<AbcG::IC3fProperty>(i_prop, i_time);
}

//-*****************************************************************************
std::string Interpolate(AbcG::IStringProperty& i_prop, chrono_t i_time) {
    return InterpolateOther<AbcG::IStringProperty>(i_prop, i_time);
}

//-*****************************************************************************
M44d Interpolate(AbcG::IXformSchema& iXform, chrono_t iTime) {
    // Get samples
    size_t numSamps = iXform.getNumSamples();
    AbcG::TimeSamplingPtr tsmp = iXform.getTimeSampling();
    std::pair<index_t, chrono_t> idx0 = tsmp->getFloorIndex(iTime, numSamps);
    std::pair<index_t, chrono_t> idx1 = tsmp->getCeilIndex(iTime, numSamps);
    chrono_t interp =
      (iTime - idx0.second) / std::max(idx1.second - idx0.second, 0.00001);

    AbcG::XformSample samp0;
    iXform.get(samp0, idx0.first);
    AbcG::XformSample samp1;
    iXform.get(samp1, idx1.first);

    auto const matrix_0 = samp0.getMatrix();
    auto rigid_matrix_0 = matrix_0;
    V3d scale_0;
    V3d shear_0;
    extractAndRemoveScalingAndShear(rigid_matrix_0, scale_0, shear_0);
    auto const translation_0 = rigid_matrix_0.translation();
    auto const quat_0 = extractQuat(rigid_matrix_0);

    auto const matrix_1 = samp1.getMatrix();
    auto rigid_matrix_1 = matrix_1;
    V3d scale_1;
    V3d shear_1;
    extractAndRemoveScalingAndShear(rigid_matrix_1, scale_1, shear_1);
    auto const translation_1 = rigid_matrix_1.translation();
    auto const quat_1 = extractQuat(rigid_matrix_1);

    auto const translation = lerp(translation_0, translation_1, interp);
    auto const quat = slerp(quat_0, quat_1, interp);
    auto const scale = lerp(scale_0, scale_1, interp);

    M44d S;
    S.setScale(scale);
    M44d const R = quat.toMatrix44();
    M44d T;
    T.setTranslation(translation);

    return S * R * T;
}

}  // End namespace AbcMeshesScene
}  // End namespace EmldCore
