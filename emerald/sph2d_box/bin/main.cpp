#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/sph2d_box/bin/multi_scale_draw.h>
#include <emerald/sph2d_box/dfsph.h>
#include <emerald/sph2d_box/iisph.h>
#include <emerald/sph2d_box/iisph_ap.h>
#include <emerald/sph2d_box/iisph_pseudo_ap.h>
#include <emerald/sph2d_box/parameters.h>
#include <emerald/sph2d_box/simulation.h>
#include <emerald/util/assert.h>
#include <emerald/util/functions.h>

#include <OpenEXR/ImathFrustum.h>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;
using namespace emerald::sph2d_box;

//------------------------------------------------------------------------------
class Sph2d_box_sim : public Sim3D {
protected:
    void init_draw(int w, int h) override {
        Sim3D::init_draw(w, h);

        m_multi_scale_draw = std::make_unique<Multi_scale_draw>(1, 2048);
        m_multi_scale_draw->update_scale(0,
                                         m_sim.config.draw_radius,
                                         m_sim.state.positions.size(),
                                         m_sim.state.positions.data(),
                                         m_sim.state.colors.data());

        m_solids_multi_scale_draw = std::make_unique<Multi_scale_draw>(1, 2048);
        m_solids_multi_scale_draw->update_scale(
          0,
          m_sim.config.draw_radius,
          m_sim.solid_state.positions.size(),
          m_sim.solid_state.positions.data(),
          m_sim.solid_state.colors.data());

        m_updated = true;
    }

public:
    explicit Sph2d_box_sim(emerald::sph2d_box::Parameters const& params)
      : Sim3D()
      , m_sim(params) {
        auto const border_size = params.support * 3.0f;

        Imath::Frustumf frustum;
        frustum.set(-1.0f,
                    1.0f,
                    -border_size,
                    m_sim.config.params.length + border_size,
                    m_sim.config.params.length + border_size,
                    -border_size,
                    true);

        m_modelview.makeIdentity();
        m_projection = frustum.projectionMatrix();
    }

    std::string name() const override {
        return "Sph2d Box Sim";
    }

    void step() override {
        // m_sim.step();
        // m_sim.state = dfsph_simulation_step(m_sim.config,
        //                                     std::move(m_sim.state),
        //                                     m_sim.solid_state,
        //                                     m_sim.temp_data);
        // m_sim.state = iisph_simulation_step(m_sim.config,
        //                                     std::move(m_sim.state),
        //                                     m_sim.solid_state,
        //                                     m_sim.temp_data);
        // m_sim.state = iisph_ap_simulation_step(m_sim.config,
        //                                        std::move(m_sim.state),
        //                                        m_sim.solid_state,
        //                                        m_sim.temp_data);
        m_sim.state = iisph_pseudo_ap_simulation_step(m_sim.config,
                                                      std::move(m_sim.state),
                                                      m_sim.solid_state,
                                                      m_sim.temp_data);

        m_multi_scale_draw->update_scale(0,
                                         m_sim.config.draw_radius,
                                         m_sim.state.positions.size(),
                                         m_sim.state.positions.data(),
                                         m_sim.state.colors.data());
        m_updated = true;
    }

    V2i preferred_window_size() const override {
        return V2i{512, 512};
    }

    bool needs_redraw() override {
        return m_updated;
    }

    void draw() override {
        glDisable(GL_DEPTH_TEST);
        m_multi_scale_draw->draw(m_modelview, m_projection);
        m_solids_multi_scale_draw->draw(m_modelview, m_projection);
        m_updated = false;
    }

    // void character(unsigned int i_char, int x, int y) override {
    //     int new_zoom_out = m_zoom_out;
    //     if (i_char == 45) {
    //         new_zoom_out = std::clamp(m_zoom_out + 1, 0, 255);
    //     } else if (i_char == 61) {
    //         new_zoom_out = std::clamp(m_zoom_out - 1, 0, 255);
    //     }

    //     if (new_zoom_out != m_zoom_out) {
    //         m_zoom_out = new_zoom_out;
    //         auto const scale = std::pow(1.1f,
    //         -static_cast<float>(m_zoom_out)); m_modelview.setScale(V3f{scale,
    //         scale, 1.0f}); m_updated = true;
    //     }
    // }

protected:
    EZ_EXAMPLE_SIM m_sim;

    M44f m_modelview;
    M44f m_projection;

    // int m_zoom_out = 0;

    std::unique_ptr<Multi_scale_draw> m_multi_scale_draw;
    std::unique_ptr<Multi_scale_draw> m_solids_multi_scale_draw;
    bool m_updated = false;
};

//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    auto const [params, num_batch_frames] = parse_parameters(argc, argv);
    SimPtr sptr = std::make_shared<Sph2d_box_sim>(params);
    SimpleViewSim(sptr, false);
    return 0;
}