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

#include "Sim.h"

namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
BaseSim::~BaseSim() {}

//-*****************************************************************************
void BaseSim::outerDraw()
{
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
    UtilGL::CheckErrors( "outerDraw glClearColor" );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    UtilGL::CheckErrors( "outerDraw glClear" );

    draw();
}

//-*****************************************************************************
Sim3D::~Sim3D() {}

//-*****************************************************************************
void Sim3D::init(int w, int h) {
    m_camera.setSize( w, h );
    m_camera.lookAt( V3d( 24, 18, 24 ), V3d( 0.0 ) );
    m_camera.frame( getBounds() );
}

//-*****************************************************************************
void Sim3D::reshape(int w, int h) {
    m_camera.setSize( w, h );
}

//-*****************************************************************************
void Sim3D::frame() {
    m_camera.frame( getBounds() );
}

//-*****************************************************************************
void Sim3D::dolly(float dx, float dy) {
    m_camera.dolly( V2d( dx, dy ) );
}

//-*****************************************************************************
void Sim3D::track(float dx, float dy) {
    m_camera.track( V2d( dx, dy ) );
}

//-*****************************************************************************
void Sim3D::rotate(float dx, float dy) {
    m_camera.rotate( V2d( dx, dy ) );
}

//-*****************************************************************************
void Sim3D::outputCamera() {
    std::cout << "# Camera\n" << m_camera.RIB() << std::endl;
}

//-*****************************************************************************
void Sim3D::outerDraw()
{
#if OSX_GLFW_VIEWPORT_BUG
    glViewport( 0, 0,
                2*( GLsizei )m_camera.width(),
                2*( GLsizei )m_camera.height() );
#else
    glViewport( 0, 0,
                ( GLsizei )m_camera.width(),
                ( GLsizei )m_camera.height() );
#endif

    double n, f;
    m_camera.autoSetClippingPlanes( getBounds() );
    if ( overrideClipping( n, f ) )
    {
        m_camera.setClippingPlanes( n, f );
    }

    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
    UtilGL::CheckErrors( "outerDraw glClearColor" );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    UtilGL::CheckErrors( "outerDraw glClear" );

    draw();
}

} // End namespace SimpleSimViewer
} // End namespace EmldCore