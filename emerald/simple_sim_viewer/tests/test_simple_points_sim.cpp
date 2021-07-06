#include <emerald/geep_glfw/program.h>
#include <emerald/simple_sim_viewer/points_draw_helper.h>
#include <emerald/simple_sim_viewer/std_shaders.h>
#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/util/assert.h>
#include <emerald/util/functions.h>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;

//-*****************************************************************************
class SimplePointsSim : public Sim3D {
protected:
    void setTime(double i_time) {
        m_time = i_time;
        m_angleOfRotation = m_time * m_rotationRate;
        m_objectToWorld.setAxisAngle(m_axisOfRotation, m_angleOfRotation);
        m_worldSpaceBounds =
            Imath::transform(m_objectSpaceBounds, m_objectToWorld);
        m_updated = true;
    }

    void init_draw(int w, int h) override {
        Sim3D::init_draw(w, h);

        m_pointsDrawHelper =
            std::make_unique<PointsDrawHelper>(false,
                                               m_vtxPosData.size(),
                                               m_vtxPosData.data(),
                                               m_vtxNormData.data(),
                                               nullptr,
                                               nullptr);

        Program::Bindings vtxBindings;
        vtxBindings.push_back(
            Program::Binding(m_pointsDrawHelper->posVboIdx(), "g_Pobj"));
        vtxBindings.push_back(
            Program::Binding(m_pointsDrawHelper->normVboIdx(), "g_Nobj"));

        Program::Bindings frgBindings;
        frgBindings.push_back(Program::Binding(0, "g_fragmentColor"));

        m_program =
            std::make_unique<Program>("SimplePointsDraw",
                                      SimpleVertexShader_P_N(),
                                      SimplePointsGeometryShader(),
                                      KeyFillFragmentShader(),
                                      vtxBindings,
                                      frgBindings,
                                      m_pointsDrawHelper->vertexArrayObject());

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
    SimplePointsSim()
      : Sim3D()
      , m_axisOfRotation(0.0, 0.0, 1.0)
      , m_angleOfRotation(0.0)
      , m_rotationRate(0.75)
      , m_time(0.0) {
        V3d c000(-0.5, -0.5, -0.5);
        V3d c100(0.5, -0.5, -0.5);
        V3d c010(-0.5, 0.5, -0.5);
        V3d c110(0.5, 0.5, -0.5);
        V3d c001(-0.5, -0.5, 0.5);
        V3d c101(0.5, -0.5, 0.5);
        V3d c011(-0.5, 0.5, 0.5);
        V3d c111(0.5, 0.5, 0.5);

        m_vtxPosData.push_back(c000);
        m_vtxNormData.push_back(c000.normalized());

        m_vtxPosData.push_back(c100);
        m_vtxNormData.push_back(c100.normalized());

        m_vtxPosData.push_back(c010);
        m_vtxNormData.push_back(c010.normalized());

        m_vtxPosData.push_back(c110);
        m_vtxNormData.push_back(c110.normalized());

        m_vtxPosData.push_back(c001);
        m_vtxNormData.push_back(c001.normalized());

        m_vtxPosData.push_back(c101);
        m_vtxNormData.push_back(c101.normalized());

        m_vtxPosData.push_back(c011);
        m_vtxNormData.push_back(c011.normalized());

        m_vtxPosData.push_back(c111);
        m_vtxNormData.push_back(c111.normalized());

        m_objectSpaceBounds.min = c000;
        m_objectSpaceBounds.max = c111;

        setTime(m_time);
    }

    std::string name() const override {
        return "SimplePointsSim";
    }

    void step() override {
        setTime(m_time + (1.0 / 24.0));
    }

    Box3d bounds() const override {
        return m_worldSpaceBounds;
    }

    bool needs_redraw() override {
        return m_camera_changed_since_last_draw || m_updated;
    }

    //! This draws, assuming a camera matrix has already been set.
    //! ...
    void draw() override {
        if (m_program && m_pointsDrawHelper) {
            glEnable(GL_PROGRAM_POINT_SIZE);

            SetStdMatrices(*m_program, m_camera, m_objectToWorld);
            m_program->use();

            (*m_program)(
                Uniform("g_pointSize",
                        (float)25.0,
                        (Uniform::Requirement)Uniform::kRequireOptional));
            m_program->setUniforms();

            m_pointsDrawHelper->draw(m_camera);
            m_program->unuse();
            m_updated = false;
        }
    }

protected:
    std::vector<V3f> m_vtxPosData;
    std::vector<V3f> m_vtxNormData;
    M44d m_objectToWorld;
    V3d m_axisOfRotation;
    double m_angleOfRotation;
    double m_rotationRate;
    double m_time;
    Box3d m_objectSpaceBounds;
    Box3d m_worldSpaceBounds;

    bool m_updated = false;

    std::unique_ptr<PointsDrawHelper> m_pointsDrawHelper;
    std::unique_ptr<Program> m_program;
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
    SimPtr sptr = std::make_shared<SimplePointsSim>();
    SimpleViewSim(sptr, true);
    return 0;
}
