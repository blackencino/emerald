#include "TestViewScene.h"

namespace AbcmTest {

using namespace emerald::simple_sim_viewer;

//-*****************************************************************************
//-*****************************************************************************
// MESH AND DRAW
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
MeshAndDraw::MeshAndDraw(MeshHandle& i_mesh)
  : m_mesh(i_mesh) {
    if (m_mesh.mesh().isRigid()) {
        m_deform = MeshDrawHelper::kStaticDeform;
    } else if (i_mesh.mesh().isConsistent()) {
        m_deform = MeshDrawHelper::kConsistentDeform;
    } else {
        m_deform = MeshDrawHelper::kInconsistentDeform;
    }

    // Get positions, normals, and indices.
    const V3f* posData = m_mesh.mesh().triMesh()->positions().begin();
    const V3f* normData = m_mesh.mesh().triMesh()->normals().begin();
    const V3ui* idxData = reinterpret_cast<const V3ui*>(
      m_mesh.mesh().triMesh()->triIndices().begin());
    std::size_t numVerts = m_mesh.mesh().triMesh()->positions().size();
    std::size_t numTris = m_mesh.mesh().triMesh()->numTriangles();

    // Make mesh draw helper.
    m_drawHelper.reset(new MeshDrawHelper(
      m_deform, numTris, numVerts, idxData, posData, normData, NULL, NULL));
}

//-*****************************************************************************
void MeshAndDraw::update() {
    if (m_deform == MeshDrawHelper::kStaticDeform) {
        return;
    } else {
        const V3f* posData = m_mesh.mesh().triMesh()->positions().begin();
        const V3f* normData = m_mesh.mesh().triMesh()->normals().begin();

        if (m_deform == MeshDrawHelper::kConsistentDeform) {
            m_drawHelper->update(posData, normData, NULL, NULL);
        } else {
            const V3ui* idxData = reinterpret_cast<const V3ui*>(
              m_mesh.mesh().triMesh()->triIndices().begin());
            std::size_t numVerts = m_mesh.mesh().triMesh()->positions().size();
            std::size_t numTris = m_mesh.mesh().triMesh()->numTriangles();
            m_drawHelper->update(
              numTris, numVerts, idxData, posData, normData, NULL, NULL);
        }
    }
}

//-*****************************************************************************
void MeshAndDraw::draw(
  emerald::geep_glfw::Program& o_program,
  const emerald::simple_sim_viewer::GLCamera& i_cam) const {
    M44d o2w = m_mesh.mesh().localToSim();
    emerald::simple_sim_viewer::SetStdMatrices(o_program, i_cam, o2w);
    o_program.setUniforms();
    m_drawHelper->draw(i_cam);
}

//-*****************************************************************************
//-*****************************************************************************
// VIEW SCENE
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
ViewScene::ViewScene(SceneSptr i_scene, chrono_t i_increment)
  : emerald::simple_sim_viewer::Sim3D()
  , m_scene(i_scene)
  , m_increment(i_increment) {
    // Nothing. MeshAndDraws will be built on initDraw
    m_simName = "Sim";
}

//-*****************************************************************************
void ViewScene::step() {
    static char buf[512];

    if (m_scene->maxTime() > m_scene->minTime()) {
        chrono_t i_time = m_scene->currentTime();
        i_time += m_increment;

        if (i_time > m_scene->maxTime() || i_time < m_scene->minTime()) {
            i_time = m_scene->minTime();
        }
        m_scene->setTime(i_time);

        snprintf(buf, 511, "Sim - Time %f", i_time);
        m_simName = buf;
    }

    // Update the mesh and draws. This will only do work on the deforming
    // ones.
    for (MeshAndDrawSptrVector::iterator miter = m_meshAndDraws.begin();
         miter != m_meshAndDraws.end();
         ++miter) {
        (*miter)->update();
    }
}

//-*****************************************************************************
Box3d ViewScene::bounds() const {
    return m_scene->simBounds();
}

//-*****************************************************************************
void ViewScene::draw() {
    m_program->use();

    for (MeshAndDrawSptrVector::const_iterator miter = m_meshAndDraws.begin();
         miter != m_meshAndDraws.end();
         ++miter) {
        (*miter)->draw(*m_program, m_camera);
    }

    m_program->unuse();
}

//-*****************************************************************************
void ViewScene::init_draw(int w, int h) {
    emerald::simple_sim_viewer::Sim3D::init_draw(w, h);

    // Build meshes.
    std::size_t numMeshes = m_scene->numMeshHandles();
    for (std::size_t i = 0; i < numMeshes; ++i) {
        MeshHandle& mesh = m_scene->getMeshHandle(i);
        std::size_t numVerts = mesh.mesh().triMesh()->positions().size();
        std::size_t numTris = mesh.mesh().triMesh()->numTriangles();
        if (numVerts > 0 && numTris > 0) {
            MeshAndDrawSptr ms(new MeshAndDraw(mesh));
            m_meshAndDraws.push_back(ms);
        }
    }

    // If no meshes, no program.
    if (m_meshAndDraws.size() == 0) { return; }

    // Build the program.
    const emerald::simple_sim_viewer::MeshDrawHelper& mdh0 =
      m_meshAndDraws.front()->drawHelper();

    emerald::geep_glfw::Program::Bindings vtxBindings;
    vtxBindings.push_back(
      emerald::geep_glfw::Program::Binding(mdh0.posVboIdx(), "g_Pobj"));
    vtxBindings.push_back(
      emerald::geep_glfw::Program::Binding(mdh0.normVboIdx(), "g_Nobj"));

    emerald::geep_glfw::Program::Bindings frgBindings;
    frgBindings.push_back(
      emerald::geep_glfw::Program::Binding(0, "g_fragmentColor"));

    m_program.reset(new emerald::geep_glfw::Program(
      "SimpleMeshDraw",
      emerald::simple_sim_viewer::SimpleVertexShader(),
      emerald::simple_sim_viewer::SimpleTrianglesGeometryShader(),
      emerald::simple_sim_viewer::KeyFillFragmentShader(),
      // emerald::simple_sim_viewer::ConstantRedFragmentShader(),
      vtxBindings,
      frgBindings,
      mdh0.vertexArrayObject()));

    // Sun, moon.
    float sunAltitude = radians(45.0f);
    float sunAzimuth = radians(35.0f);
    V3f toSun(cosf(sunAltitude) * cosf(sunAzimuth),
              cosf(sunAltitude) * sinf(sunAzimuth),
              sinf(sunAltitude));
    V3f sunColor(1.0f, 1.0f, 1.0f);

    float moonAltitude = radians(65.0f);
    float moonAzimuth = sunAzimuth - radians(180.0f);
    V3f toMoon(cosf(moonAltitude) * cosf(moonAzimuth),
               cosf(moonAltitude) * sinf(moonAzimuth),
               sinf(moonAltitude));
    V3f moonColor(0.1f, 0.1f, 0.3f);

    SetKeyFillLights(*m_program, toSun, sunColor, toMoon, moonColor);

    // Materials.
    SetStdMaterial(*m_program, V3f(0.18f), 0.0f, V3f(1.0f), 25.0f);
}

}  // End namespace AbcmTest
