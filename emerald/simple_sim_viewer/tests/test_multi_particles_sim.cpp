#include "multi_scale_draw.h"

#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/util/assert.h>
#include <emerald/util/functions.h>

#include <tbb/parallel_for.h>

#include <random>
#include <vector>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;

// static float wrap_angle(float const rad) {
//     static constexpr float pi2 = static_cast<float>(M_PI * 2.0f);
//     float const ret = std::fmod(rad, pi2);
//     return (ret < 0.0f) ? ret + pi2 : ret;
// }

struct Single_scale_particles {
    std::vector<V2f> centers;
    std::vector<float> angles;
    std::vector<C4c> rgbas;
    float radius = 1.0f;

    Single_scale_particles(std::mt19937& gen,
                           std::uniform_real_distribution<float> center_dist,
                           size_t const count,
                           float const _radius)
      : radius(_radius) {
        std::uniform_real_distribution<float> angle_dist{0.0f, radians(360.0f)};
        std::uniform_int_distribution<int> color_dist{64, 255};

        centers.reserve(count);
        angles.reserve(count);
        rgbas.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            centers.push_back({center_dist(gen), center_dist(gen)});
            angles.push_back(angle_dist(gen));
            rgbas.push_back({static_cast<uint8_t>(color_dist(gen)),
                             static_cast<uint8_t>(color_dist(gen)),
                             static_cast<uint8_t>(color_dist(gen)),
                             255});
        }
    }

    void step() {
        tbb::parallel_for(size_t{0}, centers.size(), [this](auto const i) {
            auto const angle =
              angles[i];  // auto const angle = wrap_angle(angles[i] +
                          // radians(0.1f));
            auto const ca = std::cos(angle);
            auto const sa = std::sin(angle);
            centers[i] += 0.005f * radius * V2f{ca, sa};
            // angles[i] = angle;
        });
    }
};

//-*****************************************************************************
class Multi_particles_sim : public Sim3D {
protected:
    void init_draw(int w, int h) override {
        Sim3D::init_draw(w, h);

        m_multi_scale_draw =
          std::make_unique<Multi_scale_draw>(m_scales.size(), 2048);
        for (size_t i = 0; i < m_scales.size(); ++i) {
            auto const& scale = m_scales[i];
            m_multi_scale_draw->update_scale(static_cast<int>(i),
                                             scale.radius,
                                             scale.centers.size(),
                                             scale.centers.data(),
                                             scale.angles.data(),
                                             scale.rgbas.data());
        }
        m_updated = true;
    }

public:
    Multi_particles_sim()
      : Sim3D() {
        std::mt19937 gen{54321};

        int const scale_count = 16;
        m_scales.reserve(scale_count);
        for (int i = 0; i < scale_count; ++i) {
            float const range =
              std::pow(2.0f, static_cast<float>((scale_count - 1) - i));

            m_scales.emplace_back(
              gen,
              std::uniform_real_distribution<float>{-range, range},
              1500,
              range * 0.01f);
        }

        m_modelview.makeIdentity();
        m_projection.makeIdentity();
    }

    std::string name() const override {
        return "Multi Particles Sim";
    }

    void step() override {
        for (size_t i = 0; i < m_scales.size(); ++i) {
            auto& scale = m_scales[i];
            scale.step();
            m_multi_scale_draw->update_scale(static_cast<int>(i),
                                             scale.radius,
                                             scale.centers.size(),
                                             scale.centers.data(),
                                             scale.angles.data(),
                                             scale.rgbas.data());
        }
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
        m_updated = false;
    }

    void character(unsigned int i_char, int /*x*/, int /*y*/) override {
        int new_zoom_out = m_zoom_out;
        if (i_char == 45) {
            new_zoom_out = std::clamp(m_zoom_out + 1, 0, 255);
        } else if (i_char == 61) {
            new_zoom_out = std::clamp(m_zoom_out - 1, 0, 255);
        }

        if (new_zoom_out != m_zoom_out) {
            m_zoom_out = new_zoom_out;
            auto const scale = std::pow(1.1f, -static_cast<float>(m_zoom_out));
            m_modelview.setScale(V3f{scale, scale, 1.0f});
            m_updated = true;
        }
    }

protected:
    std::vector<Single_scale_particles> m_scales;

    M44f m_modelview;
    M44f m_projection;

    int m_zoom_out = 0;

    std::unique_ptr<Multi_scale_draw> m_multi_scale_draw;
    bool m_updated = false;
};

//-*****************************************************************************
int main(int /*argc*/, char* /*argv*/[]) {
    SimPtr sptr = std::make_shared<Multi_particles_sim>();
    SimpleViewSim(sptr, true);
    return 0;
}
