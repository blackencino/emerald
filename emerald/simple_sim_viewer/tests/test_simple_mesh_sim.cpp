#include <OpenEXR/ImathMatrixAlgo.h>
#include <emerald/geep_glfw/program.h>
#include <emerald/simple_sim_viewer/mesh_draw_helper.h>
#include <emerald/simple_sim_viewer/std_shaders.h>
#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/util/assert.h>
#include <emerald/util/functions.h>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;

//-*****************************************************************************
class SimpleMeshSim : public Sim3D {
protected:
    void setTime(double i_time) {
        m_time = i_time;
        m_angleOfRotation = m_time * m_rotationRate;
        m_objectToWorld.setAxisAngle(m_axisOfRotation, m_angleOfRotation);
        m_worldSpaceBounds =
            Imath::transform(m_objectSpaceBounds, m_objectToWorld);

        m_updated = true;
    }

    void pushBackQuad(const V3f& i_c00,
                      const V3f& i_c10,
                      const V3f& i_c01,
                      const V3f& i_c11,
                      const V3f& i_n) {
        // Pushing back two triangles.
        V3ui triA;

        triA[0] = m_vtxPosData.size();
        m_vtxPosData.push_back(i_c00);
        m_vtxNormData.push_back(i_n);

        triA[1] = m_vtxPosData.size();
        m_vtxPosData.push_back(i_c10);
        m_vtxNormData.push_back(i_n);

        triA[2] = m_vtxPosData.size();
        m_vtxPosData.push_back(i_c01);
        m_vtxNormData.push_back(i_n);

        m_triIndices.push_back(triA);

        V3ui triB;

        triB[0] = m_vtxPosData.size();
        m_vtxPosData.push_back(i_c01);
        m_vtxNormData.push_back(i_n);

        triB[1] = m_vtxPosData.size();
        m_vtxPosData.push_back(i_c10);
        m_vtxNormData.push_back(i_n);

        triB[2] = m_vtxPosData.size();
        m_vtxPosData.push_back(i_c11);
        m_vtxNormData.push_back(i_n);

        m_triIndices.push_back(triB);
    }

    virtual void init_draw(int w, int h) override {
        Sim3D::init_draw(w, h);

        m_meshDrawHelper =
            std::make_unique<MeshDrawHelper>(MeshDrawHelper::kStaticDeform,
                                             m_triIndices.size(),
                                             m_vtxPosData.size(),
                                             m_triIndices.data(),
                                             m_vtxPosData.data(),
                                             m_vtxNormData.data(),
                                             nullptr,
                                             nullptr);

        Program::Bindings vtxBindings;
        vtxBindings.emplace_back(m_meshDrawHelper->posVboIdx(), "g_Pobj");
        vtxBindings.emplace_back(m_meshDrawHelper->normVboIdx(), "g_Nobj");

        Program::Bindings frgBindings;
        frgBindings.emplace_back(0, "g_fragmentColor");

        m_program =
            std::make_unique<Program>("SimpleMeshDraw",
                                      SimpleVertexShader_P_N(),
                                      SimpleTrianglesGeometryShader(),
                                      KeyFillFragmentShader(),
                                      vtxBindings,
                                      frgBindings,
                                      m_meshDrawHelper->vertexArrayObject());

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
        SetStdMaterial(*m_program, V3f(0.18f), 0.0f, V3f(0.1f), 25.0f);

        m_updated = true;
    }

public:
    SimpleMeshSim()
      : Sim3D()
      , m_axisOfRotation(0.0, 0.0, 1.0)
      , m_angleOfRotation(0.0)
      , m_rotationRate(0.0)
      , m_time(0.0) {
        V3d c000(-0.5, -0.5, -0.5);
        V3d c100(0.5, -0.5, -0.5);
        V3d c010(-0.5, 0.5, -0.5);
        V3d c110(0.5, 0.5, -0.5);
        V3d c001(-0.5, -0.5, 0.5);
        V3d c101(0.5, -0.5, 0.5);
        V3d c011(-0.5, 0.5, 0.5);
        V3d c111(0.5, 0.5, 0.5);

        V3d nNegX(-1.0, 0.0, 0.0);
        V3d nPosX(1.0, 0.0, 0.0);
        V3d nNegY(0.0, -1.0, 0.0);
        V3d nPosY(0.0, 1.0, 0.0);
        V3d nNegZ(0.0, 0.0, -1.0);
        V3d nPosZ(0.0, 0.0, 1.0);

        pushBackQuad(c010, c000, c011, c001, nNegX);
        pushBackQuad(c100, c110, c101, c111, nPosX);
        pushBackQuad(c000, c100, c001, c101, nNegY);
        pushBackQuad(c110, c010, c111, c011, nPosY);
        pushBackQuad(c100, c000, c110, c010, nNegZ);
        pushBackQuad(c001, c101, c011, c111, nPosZ);

        m_objectSpaceBounds.min = c000;
        m_objectSpaceBounds.max = c111;

        setTime(m_time);
    }

    virtual std::string name() const override {
        return "SimpleMeshSim";
    }

    virtual void step() override {
        setTime(m_time + (1.0 / 24.0));
    }

    virtual Box3d bounds() const override {
        return m_worldSpaceBounds;
    }

    bool needs_redraw() override {
        return m_camera_changed_since_last_draw || m_updated;
    }

    //! This draws, assuming a camera matrix has already been set.
    //! ...
    virtual void draw() override {
        SetStdMatrices(*m_program, m_camera, m_objectToWorld);
        m_program->use();
        m_meshDrawHelper->draw(m_camera);
        m_program->unuse();
        m_updated = false;
    }

protected:
    std::vector<V3f> m_vtxPosData;
    std::vector<V3f> m_vtxNormData;
    std::vector<V3ui> m_triIndices;
    M44d m_objectToWorld;
    V3d m_axisOfRotation;
    double m_angleOfRotation;
    double m_rotationRate;
    double m_time;
    Box3d m_objectSpaceBounds;
    Box3d m_worldSpaceBounds;

    bool m_updated = false;

    std::unique_ptr<MeshDrawHelper> m_meshDrawHelper;
    std::unique_ptr<Program> m_program;
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
    auto sptr = std::make_shared<SimpleMeshSim>();
    SimpleViewSim(sptr, true);
    return 0;
}
