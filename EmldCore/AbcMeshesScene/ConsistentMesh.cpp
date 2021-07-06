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

#include "ConsistentMesh.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
ConsistentMesh::ConsistentMesh(Object& i_enclosingObject,
                               AbcG::IPolyMesh& i_abcPolyMesh,
                               Scene& i_scene)
  : Mesh(i_enclosingObject, i_abcPolyMesh, i_scene) {
    // Set the time of self - this is non-recursive.
    evalTime(m_enclosingObject.currentTime());
}

//-*****************************************************************************
void ConsistentMesh::setTime() {
    Mesh::setTime();
    evalTime(m_enclosingObject.currentTime());
}

//-*****************************************************************************
void ConsistentMesh::evalTime(chrono_t i_time) {
    // Get Schema.
    AbcG::IPolyMeshSchema& mSchema = m_abcPolyMesh.getSchema();
    ABCM_ASSERT(m_abcPolyMeshVariance == AbcG::kHomogenousTopology,
                "Consistent mesh must have homogeneous topology");

    // Interpolate samples manually.
    size_t numSamps = mSchema.getNumSamples();
    AbcG::TimeSamplingPtr tsmp = mSchema.getTimeSampling();
    std::pair<index_t, chrono_t> idx0 = tsmp->getFloorIndex(i_time, numSamps);
    std::pair<index_t, chrono_t> idx1 = tsmp->getCeilIndex(i_time, numSamps);
    chrono_t interp = 0.0;
    if (idx1.second > idx0.second) {
        interp = (i_time - idx0.second) / (idx1.second - idx0.second);
    }

    // Get start and end samples.
    AbcG::IPolyMeshSchema::Sample samp0;
    mSchema.get(samp0, idx0.first);
    AbcG::IPolyMeshSchema::Sample samp1;
    mSchema.get(samp1, idx1.first);

    // Get points.
    Etm::ConstV3fRange pointsRange0(
      samp0.getPositions()->get(),
      samp0.getPositions()->get() + samp0.getPositions()->size());
    Etm::ConstV3fRange pointsRange1(
      samp1.getPositions()->get(),
      samp1.getPositions()->get() + samp1.getPositions()->size());
    ABCM_ASSERT(samp0.getPositions()->size() == samp1.getPositions()->size(),
                "Mismatched positions");

    if (!m_triMesh) {
        Etm::ConstIntRange indicesRange(
          samp0.getFaceIndices()->get(),
          samp0.getFaceIndices()->get() + samp0.getFaceIndices()->size());
        Etm::ConstIntRange countsRange(
          samp0.getFaceCounts()->get(),
          samp0.getFaceCounts()->get() + samp0.getFaceCounts()->size());
        m_triMesh.reset(new TriMesh(m_abcPolyMesh.getName(),
                                    pointsRange0,
                                    indicesRange,
                                    countsRange,
                                    m_localToInternal));
    }

    m_triMesh->rebuild(pointsRange0,
                       pointsRange1,
                       static_cast<float>(interp),
                       m_localToInternal);

    Box3f tmb = m_triMesh->bounds();
    if (tmb.isEmpty()) {
        m_internalBounds.makeEmpty();
        // std::cout << "Cdef TriMesh bounds are empty" << std::endl;
    } else {
        m_internalBounds.min = V3d(tmb.min);
        m_internalBounds.max = V3d(tmb.max);

        // std::cout << "Cdef TriMesh bounds " << m_internalBounds.min << " to "
        //          << m_internalBounds.max << std::endl;
    }
}

}  // End namespace AbcMeshesScene
}  // End namespace EmldCore
