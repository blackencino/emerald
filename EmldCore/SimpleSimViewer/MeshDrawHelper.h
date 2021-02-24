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

#ifndef _EmldCore_SimpleSimViewer_MeshDrawHelper_h_
#define _EmldCore_SimpleSimViewer_MeshDrawHelper_h_

#include "Foundation.h"
#include "GLCamera.h"

namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
class MeshDrawHelper
{
public:
    enum DeformType
    {
        // No changes at all
        kStaticDeform,

        // Vertex data (positions, colors, normals, etc) change, but
        // not indices
        kConsistentDeform,

        // Everything changes
        kInconsistentDeform
    };

    MeshDrawHelper
    (
        DeformType i_deformType,
        std::size_t i_numTriangles,
        std::size_t i_numVertices,
        const V3ui* i_triIndices,
        const V3f* i_vtxPosData,
        const V3f* i_vtxNormData,
        const V3f* i_vtxColData,
        const V2f* i_vtxUvData
    );

    virtual ~MeshDrawHelper();

    // Update only the vertex data, leave topology alone
    void update( const V3f* i_vtxPosData,
                 const V3f* i_vtxNormData,
                 const V3f* i_vtxColData,
                 const V2f* i_vtxUvData );

    // Update EVERYTHING.
    void update( std::size_t i_numTriangles,
                 std::size_t i_numVertices,
                 const V3ui* i_triIndices,
                 const V3f* i_vtxPosData,
                 const V3f* i_vtxNormData,
                 const V3f* i_vtxColData,
                 const V2f* i_vtxUvData );

    // Draw.
    void draw( const GLCamera& i_cam ) const;

    // Draw without camera.
    void draw() const;

    GLint posVboIdx() const { return m_posVboIdx; }
    GLint normVboIdx() const { return m_normVboIdx; }
    GLint colVboIdx() const { return m_colVboIdx; }
    GLint uvVboIdx() const { return m_uvVboIdx; }
    GLint indicesVboIdx() const { return m_indicesVboIdx; }

    GLuint vertexArrayObject() const { return m_vertexArrayObject; }

protected:
    template <typename T>
    void updateFloatVertexBuffer( const T* i_vtxData, int i_vboIdx )
    {
        if ( i_vboIdx < 0 || i_vtxData == NULL || m_numVertices == 0 )
        { return; }

        glBindBuffer( GL_ARRAY_BUFFER, m_vertexBuffers[i_vboIdx] );
        UtilGL::CheckErrors( "glBindBuffer" );
        glBufferData( GL_ARRAY_BUFFER,
                      sizeof( T ) * m_numVertices,
                      ( const GLvoid* )i_vtxData,
                      GL_DYNAMIC_DRAW );
        UtilGL::CheckErrors( "glBufferData" );
    }

    DeformType m_deformType;
    std::size_t m_numTriangles;
    std::size_t m_numVertices;
    const V3ui* m_triIndices;
    const V3f* m_vtxPosData;
    const V3f* m_vtxNormData;
    const V3f* m_vtxColData;
    const V2f* m_vtxUvData;

    // The VAO and VBOs
    GLuint m_vertexArrayObject;
    // At most we have indices, pos, norm, col, uv
    GLuint m_vertexBuffers[5];
    int m_numVBOs;
    GLint m_posVboIdx;
    GLint m_normVboIdx;
    GLint m_colVboIdx;
    GLint m_uvVboIdx;
    GLint m_indicesVboIdx;
};

} // End namespace SimpleSimViewer
} // End namespace EmldCore

#endif
