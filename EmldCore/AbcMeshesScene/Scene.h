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

#ifndef _EmldCore_AbcMeshesScene_Scene_h_
#define _EmldCore_AbcMeshesScene_Scene_h_

#include "Foundation.h"
#include "Object.h"
#include "Mesh.h"
#include "MeshKdTree.h"
#include "FileInfo.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
class Scene
{
protected:
    void loadMeshesFromFile( const FileInfo& i_fileInfo,
                             TopologyMask i_topologyMask,
                             AbcG::ReadArraySampleCachePtr i_cachePtr );
public:
    Scene()
        : m_currentTime( 0.0 )
        , m_minTime( FLT_MAX )
        , m_maxTime( -FLT_MAX )
    {
        m_simBounds.makeEmpty();
    }

    Scene( const FileInfoVec& i_fileInfo,
           TopologyMask i_topologyMask = kAnyTopology,
           chrono_t i_initTime = 0.0,
           const M44d& i_simToWorld = Imath::identity44d,
           const M44d& i_worldToSim = Imath::identity44d,
           AbcG::ReadArraySampleCachePtr i_cachePtr =
               AbcG::ReadArraySampleCachePtr() );

    size_t numMeshHandles() const { return m_meshHandles.size(); }

    MeshHandle& getMeshHandle( size_t v )
    { return *m_meshHandles[v]; }

    virtual void addMeshHandle( Mesh& iMesh );

    void setTime( chrono_t i_time );

    const Box3d& simBounds() const { return m_simBounds; }

    chrono_t currentTime() const { return m_currentTime; }
    chrono_t minTime() const { return m_minTime; }
    chrono_t maxTime() const { return m_maxTime; }

    const M44d& simToScene() const { return m_simToScene; }
    const M44d& sceneToSim() const { return m_sceneToSim; }

    //-*************************************************************************
    // REGION STUFF
    //-*************************************************************************

    bool areBoundsFullyInside( const Box3d& i_bounds ) const
    {
        if ( m_tree ) { return m_tree->areBoundsFullyInside( i_bounds ); }
        else { return false; }
    }

    bool areBoundsFullyOutside( const Box3d& i_bounds ) const
    {
        if ( m_tree ) { return m_tree->areBoundsFullyOutside( i_bounds ); }
        else { return true; }
    }

    bool intersects( const Box3d& i_bounds ) const
    {
        if ( m_tree ) { return m_tree->intersects( i_bounds ); }
        else { return false; }
    }

    bool intersects( const V3d& i_point ) const
    {
        if ( m_tree ) { return m_tree->intersects( i_point ); }
        else { return false; }
    }

    bool intersects( const V3d& i_point,
                     BestMeshPointD& o_bestMeshPoint ) const
    {
        if ( m_tree ) { return m_tree->intersects( i_point,
                                                   o_bestMeshPoint ); }
        else { return false; }
    }

    bool intersectsInnerNarrowBand( const Box3d& i_bounds,
                                    double i_narrowBand2 ) const
    {
        if ( m_tree ) 
        { 
            return m_tree->intersectsInnerNarrowBand( i_bounds,
                                                      i_narrowBand2 );
        }
        else { return false; }
    }

    bool intersectsInnerNarrowBand( const V3d& i_point,
                                    double i_narrowBand2,
                                    BestMeshPointD& o_bestMeshPoint ) const
    {
        if ( m_tree ) 
        { 
            return m_tree->intersectsInnerNarrowBand( i_point,
                                                      i_narrowBand2,
                                                      o_bestMeshPoint );
        }
        else { return false; }
    }

protected:
    chrono_t m_currentTime;
    chrono_t m_minTime;
    chrono_t m_maxTime;

    M44d m_simToScene;
    M44d m_sceneToSim;

    ObjectSptrVec m_topObjects;
    std::vector<MeshHandleSptr> m_meshHandles;
    Box3d m_simBounds;

    ABCM_UNIQUE_PTR<MeshKdTree> m_tree;
};

//-*****************************************************************************
typedef ABCM_SHARED_PTR<Scene> SceneSptr;

} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif
