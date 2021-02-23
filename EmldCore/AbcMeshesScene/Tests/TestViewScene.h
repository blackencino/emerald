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

#ifndef _EmldCore_AbcMeshesScene_TestViewScene_h_
#define _EmldCore_AbcMeshesScene_TestViewScene_h_

#include "TestFoundation.h"

namespace AbcmTest {

//-*****************************************************************************
class MeshAndDraw
{
public:
    MeshAndDraw( MeshHandle& i_mesh );

    void update();
    void draw( GeepGLFW::Program& o_program,
               const SimpleSimViewer::GLCamera& i_cam ) const;
    const SimpleSimViewer::MeshDrawHelper& drawHelper() const
    {
        return *m_drawHelper;
    }

protected:
    MeshHandle& m_mesh;
    SimpleSimViewer::MeshDrawHelper::DeformType m_deform;
    ABCM_UNIQUE_PTR<SimpleSimViewer::MeshDrawHelper> m_drawHelper;
};

typedef ABCM_SHARED_PTR<MeshAndDraw> MeshAndDrawSptr;
typedef std::vector<MeshAndDrawSptr> MeshAndDrawSptrVector;

//-*****************************************************************************
class ViewScene : public SimpleSimViewer::Sim3D
{
public:
    ViewScene( SceneSptr i_system, chrono_t i_increment );

    virtual std::string getName() const override { return m_simName; }

    virtual void init(int w, int h) override;

    virtual void step() override;

    virtual Box3d getBounds() const override;

    virtual void draw() override;

protected:
    SceneSptr m_scene;
    std::string m_simName;

    MeshAndDrawSptrVector m_meshAndDraws;
    ABCM_UNIQUE_PTR<GeepGLFW::Program> m_program;

    chrono_t m_increment;
};

} // End namespace AbcmTest

#endif
