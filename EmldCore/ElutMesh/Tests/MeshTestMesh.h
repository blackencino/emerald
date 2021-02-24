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

#ifndef _EmldCore_ElutMesh_MeshTestMesh_h_
#define _EmldCore_ElutMesh_MeshTestMesh_h_

#include "MeshTestFoundation.h"

namespace MeshTest {

//-*****************************************************************************
class Mesh
{
protected:
    void initDraw();

public:
    typedef float value_type;

    typedef Imath::Vec3<float> V3T;
    typedef Imath::Box<V3f> B3T;

    // This can be any container that matches std::vector semantics and
    // will respond to the vector_data and vector_cdata functions in the eu::
    // namespace
    typedef std::vector<V3i> V3i_vector_type;
    typedef std::vector<V3T> V3T_vector_type;
    
    Mesh()
        : m_drawInit( false )
        , m_drawWire( true )
    {
        m_bounds.makeEmpty();
    }

    void clear()
    {
        m_positions.clear();
        m_dPdus.clear();
        m_dPdvs.clear();
        m_dPdws.clear();
        m_normals.clear();
        m_triIndices.clear();
        m_bounds.makeEmpty();
    }

    V3T_vector_type& positions() { return m_positions; }
    const V3T_vector_type& positions() const { return m_positions; }

    V3T_vector_type& dPdus() { return m_dPdus; }
    const V3T_vector_type& dPdus() const { return m_dPdus; }
    V3T_vector_type& dPdvs() { return m_dPdvs; }
    const V3T_vector_type& dPdvs() const { return m_dPdvs; }
    V3T_vector_type& dPdws() { return m_dPdws; }
    const V3T_vector_type& dPdws() const { return m_dPdws; }

    V3T_vector_type& normals() { return m_normals; }
    const V3T_vector_type& normals() const { return m_normals; }

    V3i_vector_type& triIndices() { return m_triIndices; }
    const V3i_vector_type& triIndices() const { return m_triIndices; }

    void setBounds( const B3T& i_bounds ) { m_bounds = i_bounds; }

    void buildFinished() {}

    const B3T& bounds() const  { return m_bounds; }

    void draw( const essv::GLCamera &i_camera );

    void setDrawWire( bool i_dw ) { m_drawWire = i_dw; }
    bool drawWire() const { return m_drawWire; }

    void update();

protected:
    V3T_vector_type m_positions;
    V3T_vector_type m_dPdus;
    V3T_vector_type m_dPdvs;
    V3T_vector_type m_dPdws;
    V3T_vector_type m_normals;
    V3i_vector_type m_triIndices;
    B3T m_bounds;

    bool m_drawInit;
    EMLD_UNIQUE_PTR<essv::MeshDrawHelper> m_meshDrawHelper;
    EMLD_UNIQUE_PTR<egl::Program> m_program;
    EMLD_UNIQUE_PTR<egl::Program> m_wireProgram;
    bool m_drawWire;
};
    
} // End namespace MeshTest

#endif
