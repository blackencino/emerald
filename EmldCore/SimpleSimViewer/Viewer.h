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

#ifndef _EmldCore_SimpleSimViewer_Viewer_h_
#define _EmldCore_SimpleSimViewer_Viewer_h_

#error "THIS IS DEPRECATED"

#include "Foundation.h"
#include "Sim.h"
#include "GLCamera.h"

namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
// These are basically just bits of global data that GLUT needs to work with.
struct State
{
    unsigned char bMask;
    bool showHelp;
    int last_x;
    int last_y;
    int mods;

    float pointSize;

    bool anim;

    GLCamera cam;
    SimPtr sim;


    State()
        : bMask( 0 )
        , showHelp( false )
        , last_x( 0 )
        , last_y( 0 )
        , mods( 0 )
        , pointSize( 1.0f )
        , anim( false )
        , cam()
        , sim()
    {}
};

//-*****************************************************************************
void SimpleViewSim( SimPtr sim, bool i_playing = false );

//-*****************************************************************************
void SimpleInitSim( SimPtr sim, bool i_playing = false );
void SimpleViewSim();

} // End namespace SimpleSimViewer
} // End namespace EmldCore

#endif
