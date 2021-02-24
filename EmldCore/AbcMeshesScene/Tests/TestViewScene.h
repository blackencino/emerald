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
    void draw( emerald::geep_glfw::Program& o_program,
               const emerald::simple_sim_viewer::GLCamera& i_cam ) const;
    const emerald::simple_sim_viewer::MeshDrawHelper& drawHelper() const
    {
        return *m_drawHelper;
    }

protected:
    MeshHandle& m_mesh;
    emerald::simple_sim_viewer::MeshDrawHelper::DeformType m_deform;
    ABCM_UNIQUE_PTR<emerald::simple_sim_viewer::MeshDrawHelper> m_drawHelper;
};

typedef ABCM_SHARED_PTR<MeshAndDraw> MeshAndDrawSptr;
typedef std::vector<MeshAndDrawSptr> MeshAndDrawSptrVector;

//-*****************************************************************************
class ViewScene : public emerald::simple_sim_viewer::Sim3D
{
public:
    ViewScene( SceneSptr i_system, chrono_t i_increment );

    virtual std::string name() const override { return m_simName; }

    virtual void init_draw(int w, int h) override;

    virtual void step() override;

    virtual Box3d bounds() const override;

    virtual void draw() override;

protected:
    SceneSptr m_scene;
    std::string m_simName;

    MeshAndDrawSptrVector m_meshAndDraws;
    ABCM_UNIQUE_PTR<emerald::geep_glfw::Program> m_program;

    chrono_t m_increment;
};

} // End namespace AbcmTest

#endif
