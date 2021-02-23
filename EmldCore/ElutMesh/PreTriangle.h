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

#ifndef _EmldCore_ElutMesh_PreTriangle_h_
#define _EmldCore_ElutMesh_PreTriangle_h_

//-*****************************************************************************
// This is an implementation of the paper:
// P. Cignoni, F. Ganovelli, C. Montani, and R. Scopigno:
// Reconstruction of Topologically Correct and Adaptive Trilinear Isosurfaces,
// Computer & Graphics, Elsevier Science, 1999
// Which provides an exhaustive LUT to disambiguate isosurface triangulations
// inside a rectangular cell with iso values on the 8 corners of the cell,
// such that meshes will link together without having to explicitly march
// through them, as in Marching Cubes.
//-*****************************************************************************

#include "Foundation.h"
#include "PreVertex.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
template <typename T>
class PreTriangle
{
public:
    PreTriangle() {}
    PreTriangle( const PreVertex<T>& i_v0, 
                 const PreVertex<T>& i_v1,
                 const PreVertex<T>& i_v2 )
    {
        m_vertices[0] = i_v0;
        m_vertices[1] = i_v1;
        m_vertices[2] = i_v2;
    }

    const PreVertex<T>& vertex( int i ) const
    {
        EMLD_DEBUG_ASSERT( i >= 0 && i < 3, "out of range vertex index" );
        return m_vertices[i];
    }

    const PreVertex<T>& vertex0() const { return m_vertices[0]; }
    const PreVertex<T>& vertex1() const { return m_vertices[1]; }
    const PreVertex<T>& vertex2() const { return m_vertices[2]; }

protected:
    PreVertex<T> m_vertices[3];
};

} // End namespace ElutMesh
} // End namespace EmldCore

#endif
