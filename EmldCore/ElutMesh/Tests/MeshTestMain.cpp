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

#include "MeshTestFoundation.h"
#include "MeshTestMesh.h"
#include "MeshTestNodes.h"

#include <EmldCore/SimpleSimViewer/All.h>
#include <EmldCore/Util/All.h>

#include <vector>
#include <iostream>

namespace MeshTest {

//-*****************************************************************************
class MeshTestSim : public essv::Sim3D
{
public:
    typedef EMLD_SHARED_PTR<SphereNode> SphereNodeSptr;
    typedef std::vector<SphereNodeSptr> SphereNodeSptrs;

    explicit MeshTestSim( float i_detailSize = 0.05f )
        : essv::Sim3D()
        , m_detailSize( i_detailSize )
    {
        // Make a min node.
        m_minNode.reset( new MinNode );

        // Make a bunch of sphere nodes.
        SphereNodeSptr snode;

        snode.reset( new SphereNode( V3f( 0.0f, 0.0f, 0.0f ), 10.0f ) );
        m_sphereNodes.push_back( snode );

#if 1
        snode.reset( new SphereNode( V3f( 9.0f, 4.0f, 2.0f ), 7.0f ) );
        m_sphereNodes.push_back( snode );

        snode.reset( new SphereNode( V3f( -1.0f, -11.0f, 3.0f ), 8.0f ) );
        m_sphereNodes.push_back( snode );

        snode.reset( new SphereNode( V3f( 6.0f, 6.0f, -9.0f ), 9.0f ) );
        m_sphereNodes.push_back( snode );
#endif

        // Loop over sphere nodes, gather bounds, add to min node.
        m_bounds.makeEmpty();
        for ( SphereNodeSptrs::iterator siter = m_sphereNodes.begin();
              siter != m_sphereNodes.end(); ++siter )
        {
            m_bounds.extendBy( ( *siter )->bounds() );
            m_minNode->addNode( ( *siter ).get() );
        }
        m_bounds.min -= V3f( 4.0f * m_detailSize );
        m_bounds.max += V3f( 4.0f * m_detailSize );
        std::cout << "Main bounds: " << m_bounds << std::endl;

        // Make a dicer.
        elm::DiceRectanglef::Parameters dparams;
        dparams.center = m_bounds.center();
        dparams.size = m_bounds.size();
        dparams.rotation.setXYZVector( V3f( 0.0f ) );
        dparams.detailSize = m_detailSize;
        m_dicer = elm::DiceRectanglef( dparams );

        // Make a mesh.
        m_mesh.reset( new Mesh );

        // Refine node with dicer into mesh.
        {
            ThreadSafeCache cache;
            TimeInterval t;
            NodeWrapper nwrap( *m_minNode, t, cache );
            elm::Refinery<NodeWrapper,elm::DiceRectanglef,Mesh>
                R( nwrap, m_dicer, *m_mesh );
            R.refine();
        }

        // Set bounds equal to mesh bounds
        m_bounds = m_mesh->bounds();
    }

    virtual std::string getName() const override { return "MeshTestSim"; }

    virtual void step() override {}

    virtual Box3d getBounds() const override
    {
        return Box3d( V3d( m_bounds.min ), V3d( m_bounds.max ) );
    }

    virtual void draw() override
    { m_mesh->draw( m_camera ); }

    //! This calls the character function
    virtual void character( unsigned int i_char, int x, int y ) override
    {
        switch ( i_char )
        {
        case 'w':
            m_mesh->setDrawWire( !( m_mesh->drawWire() ) );
            break;
        };

        essv::Sim3D::character( i_char, x, y );
    }

    //-*****************************************************************************
    virtual void keyboard( int i_key, int i_scancode, int i_action, int i_mods,
                           int i_x, int i_y ) override
    {
        // Don't bother with releases.
        if ( i_action == GLFW_RELEASE ) { return; }

        // Shift
        bool shift = ( bool )( i_mods & GLFW_MOD_SHIFT );

        // Control
        bool ctrl = ( bool )( i_mods & GLFW_MOD_CONTROL );

        // Alt
        bool alt = ( bool )( i_mods & GLFW_MOD_ALT );

        // Super
        bool super = ( bool )( i_mods & GLFW_MOD_SUPER );

        bool doEditMesh = false;
        switch ( i_key )
        {
            // Right arrow
        case 262:
            m_sphereNodes[0]->moveUp();
            rebuildMesh();
            break;

            // Left arrow
        case 263:
            m_sphereNodes[0]->moveDown();
            rebuildMesh();
            break;
        };

        essv::Sim3D::keyboard( i_key, i_scancode, i_action, i_mods,
                             i_x, i_y );
    }

    //-*****************************************************************************
    void rebuildMesh()
    {
        // Refine node with dicer into mesh.
        {
            ThreadSafeCache cache;
            TimeInterval t;
            NodeWrapper nwrap( *m_minNode, t, cache );
            elm::Refinery<NodeWrapper,elm::DiceRectanglef,Mesh>
                R( nwrap, m_dicer, *m_mesh );
            R.refine();
        }

        // Update mesh.
        m_mesh->update();

        // Set bounds equal to mesh bounds
        m_bounds = m_mesh->bounds();
    }

protected:
    float m_detailSize;
    EMLD_UNIQUE_PTR<MinNode> m_minNode;
    SphereNodeSptrs m_sphereNodes;
    elm::DiceRectanglef m_dicer;
    EMLD_UNIQUE_PTR<Mesh> m_mesh;

    Box3f m_bounds;
};

} // End namespace MeshTest

//-*****************************************************************************
namespace essv = EmldCore::SimpleSimViewer;

//-*****************************************************************************
int main( int argc, char* argv[] )
{
    if ( argc > 1 &&
         ( std::string( "-h" ) == argv[1] ||
           std::string( "--help" ) == argv[1] ||
           std::string( "-help" ) == argv[1] ||
           std::string( "--h" ) == argv[1] ) )
    {
        std::cerr << "USAGE: " << argv[0] << " [optional:detailSize]"
                  << std::endl;
        std::exit( -1 );
    }

    float detailSize = 0.05f;
    if ( argc > 1 )
    {
        detailSize = std::atof( argv[1] );
    }
    std::cout << "Using detail size of: " << detailSize << std::endl;

    essv::SimPtr sptr( new MeshTest::MeshTestSim( detailSize ) );
    essv::SimpleViewSim( sptr, false );
    return 0;
}


