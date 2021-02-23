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

#include "MeshTestMesh.h"

namespace MeshTest {

//-*****************************************************************************
void Mesh::initDraw()
{
    // As long as ints are non-negative, they're bitwise identical, for the 
    // same values, as unsigned ints. Therefore, this reinterpretation is
    // valid so long as we know that the indices are non-negative. (they are)
    const V3ui* triIndices =
        reinterpret_cast<const V3ui*>( vector_cdata( m_triIndices ) );
    m_meshDrawHelper.reset(
        new essv::MeshDrawHelper( essv::MeshDrawHelper::kInconsistentDeform,
                            m_triIndices.size(),
                            m_positions.size(),
                            triIndices,
                            vector_cdata( m_positions ),
                            vector_cdata( m_normals ),
                            //NULL,
                            NULL,
                            NULL ) );

    egl::Program::Bindings vtxBindings;
    vtxBindings.push_back(
        egl::Program::Binding( m_meshDrawHelper->posVboIdx(), "g_Pobj" ) );
    vtxBindings.push_back(
        egl::Program::Binding( m_meshDrawHelper->normVboIdx(), "g_Nobj" ) );

    egl::Program::Bindings frgBindings;
    frgBindings.push_back( 
        egl::Program::Binding( 0, "g_fragmentColor" ) );

    m_program.reset(
        new egl::Program( "SimpleMeshDraw",
                     essv::SimpleVertexShader(),
                     essv::SimpleTrianglesGeometryShader(),
                     essv::KeyFillFragmentShader(),
                     //ConstantRedFragmentShader(),
                     vtxBindings,
                     frgBindings,
                     m_meshDrawHelper->vertexArrayObject() ) );

    // Sun, moon.
    float sunAltitude = radians( 45.0f );
    float sunAzimuth = radians( 35.0f );
    V3f toSun( cosf( sunAltitude ) * cosf( sunAzimuth ),
               cosf( sunAltitude ) * sinf( sunAzimuth ),
               sinf( sunAltitude ) );
    V3f sunColor( 1.0f, 1.0f, 1.0f );

    float moonAltitude = radians( 65.0f );
    float moonAzimuth = sunAzimuth - radians( 180.0f );
    V3f toMoon( cosf( moonAltitude ) * cosf( moonAzimuth ),
                cosf( moonAltitude ) * sinf( moonAzimuth ),
                sinf( moonAltitude ) );
    V3f moonColor( 0.1f, 0.1f, 0.3f );

    essv::SetKeyFillLights( *m_program, toSun, sunColor, toMoon, moonColor );

    // Materials.
    essv::SetStdMaterial( *m_program, V3f( 0.18f ), V3f( 0.1f ), 25.0f );

    //-*************************************************************************
    // Wire program
    vtxBindings.clear();
    vtxBindings.push_back(
        egl::Program::Binding( m_meshDrawHelper->posVboIdx(), "g_Pobj" ) );
    m_wireProgram.reset(
        new egl::Program( "SimpleMeshWireDraw",
                essv::SimpleVertexShader(),
                essv::SimpleTrianglesWireframeGeometryShader(),
                essv::ConstantWhiteFragmentShader(),
                vtxBindings,
                frgBindings,
                m_meshDrawHelper->vertexArrayObject() ) );
}

//-*****************************************************************************
void Mesh::draw( const essv::GLCamera& i_camera )
{
    if ( !m_drawInit )
    {
        initDraw();
        m_drawInit = true;
    }

    M44d objToWorld;
    objToWorld.makeIdentity();

    egl::Program* prog = m_drawWire ? m_wireProgram.get() : m_program.get();


    essv::SetStdMatrices( *prog, i_camera, objToWorld );
    prog->use();
    m_meshDrawHelper->draw( i_camera );
    prog->unuse();
}

//-*****************************************************************************
void Mesh::update()
{
    std::size_t numTris = m_triIndices.size();
    std::size_t numVerts = m_positions.size();

    // As long as ints are non-negative, they're bitwise identical, for the 
    // same values, as unsigned ints. Therefore, this reinterpretation is
    // valid so long as we know that the indices are non-negative. (they are)
    const V3ui* triIndices =
        reinterpret_cast<const V3ui*>( vector_cdata( m_triIndices ) );
    m_meshDrawHelper->update( numTris, numVerts, triIndices,
                              vector_cdata( m_positions ),
                              vector_cdata( m_normals ),
                              //NULL,
                              NULL,
                              NULL );
}

} // End namespace MeshTest
