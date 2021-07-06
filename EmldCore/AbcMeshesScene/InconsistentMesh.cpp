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

#include "InconsistentMesh.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
InconsistentMesh::InconsistentMesh( Object& i_enclosingObject,
                                    AbcG::IPolyMesh& i_abcPolyMesh,
                                    Scene& i_scene )
    : Mesh( i_enclosingObject, i_abcPolyMesh, i_scene )
{
    // Set the time of self - this is non-recursive.
    evalTime( m_enclosingObject.currentTime() );
}

//-*****************************************************************************
void InconsistentMesh::setTime()
{
    Mesh::setTime();
    evalTime( m_enclosingObject.currentTime() );
}

//-*****************************************************************************
void InconsistentMesh::evalTime( chrono_t i_time )
{
    // Get Schema.
    AbcG::IPolyMeshSchema& mSchema = m_abcPolyMesh.getSchema();
    ABCM_ASSERT( m_abcPolyMeshVariance == AbcG::kHeterogenousTopology,
                 "Inconsistent mesh must have heterogeneous topology" );

    // Get floor sample.
    size_t numSamps = mSchema.getNumSamples();
    AbcG::TimeSamplingPtr tsmp = mSchema.getTimeSampling();
    std::pair<index_t, chrono_t> idx =
        tsmp->getFloorIndex( i_time, numSamps );
    mSchema.get( m_floorSample, idx.first );

    // Get N
    const size_t N = m_floorSample.getPositions()->size();

    // Get connectivity.
    Etm::ConstIntRange indicesRange( m_floorSample.getFaceIndices()->get(),
                                     m_floorSample.getFaceIndices()->get() +
                                     m_floorSample.getFaceIndices()->size() );

    Etm::ConstIntRange countsRange( m_floorSample.getFaceCounts()->get(),
                                    m_floorSample.getFaceCounts()->get() +
                                    m_floorSample.getFaceCounts()->size() );

    // Get vel sample.
    m_velProp.reset();
    m_velSamplePtr.reset();
    if ( i_time != idx.second )
    {
        m_velProp = mSchema.getVelocitiesProperty();
        if ( m_velProp )
        {
            size_t numVelSamps = m_velProp.getNumSamples();
            AbcG::TimeSamplingPtr vtsmp = m_velProp.getTimeSampling();
            std::pair<index_t, chrono_t> vidx =
                vtsmp->getFloorIndex( i_time, numVelSamps );
            m_velProp.get( m_velSamplePtr, vidx.first );

            if ( !m_velSamplePtr || m_velSamplePtr->size() != N )
            {
                m_velSamplePtr.reset();
                m_velProp.reset();
            }
        }
    }

    // If no vel sample, we can put points into tri mesh directly.
    if ( !m_velSamplePtr )
    {
        m_advectedLocalPoints.clear();

        // Get positions.
        Etm::ConstV3fRange
        pointsRange( m_floorSample.getPositions()->get(),
                     m_floorSample.getPositions()->get() + N );

        m_triMesh.reset(
            new TriMesh( m_abcPolyMesh.getName(),
                         pointsRange,
                         indicesRange,
                         countsRange,
                         m_localToInternal ) );
    }
    else
    {
        // We have to advect points locally.
        float dt = static_cast<float>(i_time - idx.second);
        m_advectedLocalPoints.resize( N );
        const V3f* localPos = m_floorSample.getPositions()->get();
        const V3f* localVel = m_velSamplePtr->get();
        for ( size_t i = 0; i < N; ++i )
        {
            m_advectedLocalPoints[i] = localPos[i] + dt * localVel[i];
        }

        Etm::ConstV3fRange apointsRange(
            vector_cdata( m_advectedLocalPoints ),
            vector_cdata( m_advectedLocalPoints ) + N );

        m_triMesh.reset(
            new TriMesh( m_abcPolyMesh.getName(),
                         apointsRange,
                         indicesRange,
                         countsRange,
                         m_localToInternal ) );

        // Then tell the tri mesh to use our velocities
        m_triMesh->setVelocities( 
            Etm::ConstV3fRange( m_velSamplePtr->get(),
                                m_velSamplePtr->get() + N ) );
    }

    // Set the bounds.
    Box3f tmb = m_triMesh->bounds();
    if ( tmb.isEmpty() )
    {
        m_internalBounds.makeEmpty();
    }
    else
    {
        m_internalBounds.min = V3d( tmb.min );
        m_internalBounds.max = V3d( tmb.max );
    }
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

