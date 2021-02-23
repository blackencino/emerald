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

#ifndef _EmldCore_TriMesh_Vertex_h_
#define _EmldCore_TriMesh_Vertex_h_

#include "Foundation.h"

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
class Vertex
{
public:
    Vertex()
      : m_points()
      , m_velocities()
      , m_index( -1 ) {}
    
    Vertex( V3fVector &pnts, V3fVector &vels, int idx )
      : m_points( &pnts )
      , m_velocities( &vels )
      , m_index( idx ) {}

    // Default copy & assignment operator
    // ...

    // Don't allow direct modification of points vector
    const V3fVector &points() const { return *m_points; }
    const V3fVector &velocities() const { return *m_velocities; }

    // Const reference to position
    const V3f &position() const
    {
        return (*m_points)[m_index];
    }

    // Const reference to velocity
    const V3f &velocity() const
    {
        return (*m_velocities)[m_index];
    }

    // Editable position should be done explicitly.
    void setPosition( const V3f &np )
    {
        (*m_points)[m_index] = np;
    }

    // Editable velocity should be done explicitly.
    void setVelocity( const V3f &nv )
    {
        (*m_velocities)[m_index] = nv;
    }
    
    int index() const { return m_index; }

protected:
    V3fVector *m_points;
    V3fVector *m_velocities;
    int m_index;
};

//-*****************************************************************************
typedef std::vector<Vertex> VertexVector;

} // End namespace TriMesh
} // End namespace EmldCore

#endif
