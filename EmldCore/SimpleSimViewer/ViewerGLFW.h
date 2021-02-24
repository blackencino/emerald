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

#ifndef _EmldCore_SimpleSimViewer_ViewerGLFW_h_
#define _EmldCore_SimpleSimViewer_ViewerGLFW_h_

#include "Foundation.h"
#include "Sim.h"
#include "GLCamera.h"

namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
class Viewer;

//-*****************************************************************************
class GLFWGlobal
{
public:
    GLFWGlobal();
    ~GLFWGlobal();

    void registerWindowAndViewer( GLFWwindow* i_window,
                                  Viewer* i_viewer );

    void unregisterWindowAndAllViewers( GLFWwindow* i_window );

    Viewer* viewerFromWindow( GLFWwindow* i_window,
                              bool i_throwIfNotFound = true );

protected:
    typedef std::map<GLFWwindow*, Viewer*> WindowViewerMap;
    WindowViewerMap m_wvMap;
};

//-*****************************************************************************
class Viewer
{
public:
    static const int BMASK_LEFT = 0x1<<0;  // binary 001
    static const int BMASK_MIDDLE = 0x1<<1;  // binary 010
    static const int BMASK_RIGHT = 0x1<<2;  // binary 100

    explicit Viewer( SimPtr i_sim, bool i_anim = false );
    ~Viewer();

    void init();
    void tick( bool i_force );
    void display();
    void reshape( int i_width, int i_height );
    void keyboard( int i_key, int i_scancode, int i_action, int i_mods );
    void character( unsigned int i_char );
    void mouse( int i_button, int i_action, int i_mods );
    void mouseDrag( double x, double y );
    GLFWwindow* window() { return m_window; }

protected:
    SimPtr m_sim;

    GLFWwindow* m_window;

    int m_buttonMask;
    double m_mouseX;
    double m_mouseY;
    double m_lastX;
    double m_lastY;
    int m_keyMods;

    bool m_animating;

    //GLCamera m_camera;
    Timer m_playbackTimer;
};

//-*****************************************************************************
void SimpleViewSim( SimPtr i_sim, bool i_playing = false );


} // End namespace SimpleSimViewer
} // End namespace EmldCore

#endif
