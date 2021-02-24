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

#include "Scene.h"

#include <Alembic/AbcCoreFactory/IFactory.h>

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
// SCENE
//-*****************************************************************************

//-*****************************************************************************
void Scene::loadMeshesFromFile( const FileInfo& i_fileInfo,
                                TopologyMask i_topologyMask,
                                AbcG::ReadArraySampleCachePtr i_cachePtr )
{
    std::cout << "Loading meshes from: "
              << i_fileInfo.fileName
              << ", on = " << i_fileInfo.onRegEx
              << ", off = " << i_fileInfo.offRegEx
              << std::endl;

    // Load Input Archive
    Alembic::AbcCoreFactory::IFactory factory;
    factory.setSampleCache(i_cachePtr);
    factory.setPolicy(AbcG::ErrorHandler::kThrowPolicy);
    AbcG::IArchive inArchive = factory.getArchive(i_fileInfo.fileName);

    AbcG::IObject inTopObject = inArchive.getTop();

    // Create onOffSet
    OnOffSet onOffSet( i_fileInfo.onRegEx, i_fileInfo.offRegEx );

    ObjectSptr obj( new Object( inTopObject, onOffSet, i_topologyMask,
                                NULL, *this, m_currentTime ) );
    m_topObjects.push_back( obj );
}

//-*****************************************************************************
Scene::Scene( const FileInfoVec& i_fileInfo,
              TopologyMask i_topologyMask,
              chrono_t i_initTime,
              const M44d& i_simToScene,
              const M44d& i_sceneToSim,
              AbcG::ReadArraySampleCachePtr i_cachePtr )
    : m_currentTime( i_initTime )
    , m_simToScene( i_simToScene )
    , m_sceneToSim( i_sceneToSim )
{
    // Load meshes from scenes.
    for ( FileInfoVec::const_iterator miter = i_fileInfo.begin();
          miter != i_fileInfo.end(); ++miter )
    {
        loadMeshesFromFile( ( *miter ), i_topologyMask, i_cachePtr );
    }

    // Get bounds and time range.
    m_simBounds.makeEmpty();
    m_minTime = FLT_MAX;
    m_maxTime = -FLT_MAX;
    for ( std::vector<MeshHandleSptr>::iterator miter = m_meshHandles.begin();
          miter != m_meshHandles.end(); ++miter )
    {
        const Object& mobj = (*miter)->mesh().enclosingObject();
        m_simBounds.extendBy( mobj.simBounds() );
        m_minTime = std::min( m_minTime, mobj.minTime() );
        m_maxTime = std::max( m_maxTime, mobj.maxTime() );
    }

    // Build the tree!
    if ( m_simBounds.isEmpty() )
    {
        m_tree.reset();
    }
    else
    {
        m_tree.reset( new MeshKdTree( m_meshHandles ) );
    }
}

//-*****************************************************************************
void Scene::setTime( chrono_t i_time )
{
    m_currentTime = i_time;
    for ( ObjectSptrVec::iterator oiter = m_topObjects.begin();
          oiter != m_topObjects.end(); ++oiter )
    {
        ( *oiter )->setTime( i_time );
    }

    m_simBounds.makeEmpty();
    for ( std::vector<MeshHandleSptr>::iterator miter = m_meshHandles.begin();
          miter != m_meshHandles.end(); ++miter )
    {
        m_simBounds.extendBy( ( *miter )->
                              mesh().enclosingObject().simBounds() );
    }

    // Build the tree!
    if ( m_simBounds.isEmpty() )
    {
        m_tree.reset();
    }
    else
    {
        m_tree.reset( new MeshKdTree( m_meshHandles ) );
    }
}

//-*****************************************************************************
void Scene::addMeshHandle( Mesh& iMesh )
{
    MeshHandleSptr cptr( new MeshHandle( iMesh ) );
    iMesh.setMeshId( m_meshHandles.size() );
    m_meshHandles.push_back( cptr );
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

