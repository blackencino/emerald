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

#ifndef _EmldCore_SimpleSimViewer_Sim_h_
#define _EmldCore_SimpleSimViewer_Sim_h_

#include "Foundation.h"
#include "GLCamera.h"

namespace EmldCore {
namespace SimpleSimViewer {

//-*****************************************************************************
class BaseSim
{
public:
    //! Virtual base class for simulation.
    //! ...
    BaseSim( void ) {}

    //! Virtual destructor
    //! ...
    virtual ~BaseSim();

    //! Return a name
    //! ...
    virtual std::string getName() const { return "BaseSim"; }

    //! Preferred window size.
    //! ...
    virtual V2i preferredWindowSize() const { return V2i(800, 600); }

    //! Init draw
    //! ...
    virtual void init(int w, int h) {}

    //! Reshape
    //! ...
    virtual void reshape(int w, int h) {}

    //! Just step forward.
    //! ...
    virtual void step() {}

    //! frame
    //! ...
    virtual void frame() {}

    //! Dolly
    //! ...
    virtual void dolly(float dx, float dy) {}

    //! Track
    //! ...
    virtual void track(float dx, float dy) {}

    //! Rotate
    //! ...
    virtual void rotate(float dx, float dy) {}

    //! Output camera
    //! ...
    virtual void outputCamera() {}

    //! This draws, assuming a camera matrix has already been set.
    //! ...
    virtual void draw() {}

    //! This outer draw function sets up the drawing environment.
    //! ...
    virtual void outerDraw();

    //! This calls the character function
    virtual void character( unsigned int i_char, int x, int y ) {}

    //! This calls the keyboard function.
    virtual void keyboard( int i_key, int i_scancode, int i_action, int i_mods,
                           int i_x, int i_y ) {}

    virtual void mouse( int i_button, int i_action, int i_mods,
                        double x, double y, double lastX, double lastY ) {}

    virtual void mouseDrag( double x, double y, double lastX, double lastY ) {}
};

//-*****************************************************************************
class Sim3D : public BaseSim
{
public:
    //! Virtual base class for simulation.
    //! ...
    Sim3D( void ) : BaseSim() {}

    //! Virtual destructor
    //! ...
    virtual ~Sim3D();

    //! Return a name
    //! ...
    virtual std::string getName() const override { return "Sim3D"; }

    //! Init draw
    //! ...
    virtual void init(int w, int h) override;

    //! Reshape
    //! ...
    virtual void reshape(int w, int h) override;

    //! frame
    //! ...
    virtual void frame() override;

    //! Dolly
    //! ...
    virtual void dolly(float dx, float dy) override;

    //! Track
    //! ...
    virtual void track(float dx, float dy) override;

    //! Rotate
    //! ...
    virtual void rotate(float dx, float dy) override;

    //! Output camera
    //! ...
    virtual void outputCamera() override;

    //! Return the bounds at the current time.
    //! ...
    virtual Box3d getBounds() const
    { return Box3d( V3d( -0.1 ), V3d( 0.1 ) ); }

    //! Override clipping
    //! ...
    virtual bool overrideClipping( double& o_near, double& o_far ) const
    { return false; }

    //! This outer draw function sets up the drawing environment.
    //! ...
    virtual void outerDraw() override;

    //! Return the camera
    const GLCamera& camera() const { return m_camera; }

protected:
    GLCamera m_camera;
};

typedef EMLD_SHARED_PTR<BaseSim> SimPtr;

} // End namespace SimpleSimViewer
} // End namespace EmldCore

#endif
