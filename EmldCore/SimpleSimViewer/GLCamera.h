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

#ifndef _EmldCore_SimpleSimViewer_GLCamera_h_
#define _EmldCore_SimpleSimViewer_GLCamera_h_

#include "Foundation.h"

namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
class GLCamera
{
public:
    GLCamera();
    ~GLCamera() {};

    // Executes OpenGL commands to show the current camera
    void apply() const;

    //-*************************************************************************
    // LOCAL TRANSFORM: Get, Set
    M44d modelViewMatrix() const;
    M44d projectionMatrix() const;

    const V3d &rotation() const { return m_rotation; }
    void setRotation( const V3d &r ) { m_rotation = r; }

    const V3d &scale() const { return m_scale; }
    void setScale( const V3d &s ) { m_scale = s; }

    const V3d &translation() const { return m_translation; }
    void setTranslation( const V3d &t ) { m_translation = t; }

    double centerOfInterest() const { return m_centerOfInterest; }
    void setCenterOfInterest( double coi )
    {
        m_centerOfInterest = std::max( coi, 0.1 );
    }

    double fovy() const { return m_fovy; }
    void setFovy( double fvy ) { m_fovy = fvy; }

    int width() const { return m_size.x; }
    int height() const { return m_size.y; }

    const V2d& clippingPlanes() const { return m_clip; }
    void autoSetClippingPlanes( const Box3d &bounds );
    void setClippingPlanes( double near, double far )
    {
        m_clip.x = near;
        m_clip.y = far;
    }

    //-*************************************************************************
    // UI Actions
    void track( const V2d &point );
    void dolly( const V2d &point,
                double dollySpeed = 5.0 );
    void rotate( const V2d &point,
                 double rotateSpeed = 400.0 );

    void frame( const Box3d &bounds );

    void lookAt( const V3d &eye, const V3d &at );

    void setSize( int w, int h )
    {
        m_size.x = w;
        m_size.y = h;
    }
    void setSize( const V2i &sze )
    {
        m_size = sze;
    }

    //-*************************************************************************
    // RIB STUFF
    std::string RIB() const;

    //-*************************************************************************
    // RAY STUFF - convert a screen space point, in raster space, to a
    // ray from the eye.
    Imath::Line3d getRayThroughRasterPoint(const V2d& pt_raster) const;

protected:
    //-*************************************************************************
    // DATA
    V3d m_rotation;
    V3d m_scale;
    V3d m_translation;

    double m_centerOfInterest;
    double m_fovy;
    V2d m_clip;
    V2i m_size;
    double m_aspect;
};

} // End namespace SimpleSimViewer
} // End namespace EmldCore

#endif
