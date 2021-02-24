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

#include "RigidMesh.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
RigidMesh::RigidMesh( Object& i_enclosingObject,
                      AbcG::IPolyMesh& i_abcPolyMesh,
                      Scene& i_scene )
    : Mesh( i_enclosingObject, i_abcPolyMesh, i_scene )
{
    //std::cout << "Creating RigidMesh." << std::endl;

    // Mesh constructor already extracted scale.
    ABCM_ASSERT( m_scale.length2() > 0.0, 
                 "Should already have a non-zero scale." );

    // Get Schema.
    AbcG::IPolyMeshSchema& mSchema = m_abcPolyMesh.getSchema();
    ABCM_ASSERT( m_abcPolyMeshVariance == AbcG::kConstantTopology,
                 "Cannot have a rigid mesh with non-constant topology" );

    // Read input data. Don't need to specify a sample.
    AbcG::IPolyMeshSchema::Sample psamp;
    mSchema.get( psamp );

    // Get positions.
    Etm::ConstV3fRange pointsRange( psamp.getPositions()->get(),
                                    psamp.getPositions()->get() +
                                    psamp.getPositions()->size() );

    Etm::ConstIntRange indicesRange( psamp.getFaceIndices()->get(),
                                     psamp.getFaceIndices()->get() +
                                     psamp.getFaceIndices()->size() );

    Etm::ConstIntRange countsRange( psamp.getFaceCounts()->get(),
                                    psamp.getFaceCounts()->get() +
                                    psamp.getFaceCounts()->size() );

    m_triMesh.reset(
        new TriMesh( m_abcPolyMesh.getName(),
                     pointsRange,
                     indicesRange,
                     countsRange,
                     m_localToInternal ) );

    // Set the bounds.
    Box3f tmb = m_triMesh->bounds();
    if ( tmb.isEmpty() )
    {
        m_internalBounds.makeEmpty();
        //std::cout << "Rigid TriMesh bounds are empty." << std::endl;
    }
    else
    {
        m_internalBounds.min = V3d( tmb.min );
        m_internalBounds.max = V3d( tmb.max );
        //std::cout << "Rigid TriMesh bounds " << m_internalBounds.min << " to "
        //          << m_internalBounds.max << std::endl;
    }
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

