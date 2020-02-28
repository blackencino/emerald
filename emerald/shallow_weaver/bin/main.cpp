#include <emerald/shallow_weaver/bin/parse_parameters.h>
#include <emerald/shallow_weaver/simulation.h>
#include <emerald/simple_sim_viewer/slab_draw_helper.h>
#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/util/assert.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;
using namespace emerald::shallow_weaver;

//-*****************************************************************************
class Shallow_weaver_sim : public SimSlab {
public:
    explicit Shallow_weaver_sim(Simulation::Parameters const& params)
      : SimSlab()
      , m_simulation(params) {
        m_image.resize(params.resolution * params.resolution, V3f{0.0f});
        copy_state_to_image();
    }

    void copy_state_to_image() {
        V3f const dark_blue{0.01f, 0.01f, 0.2f};
        V3f const light_blue{0.9f, 0.9f, 1.0f};

        auto const data_span = m_simulation.state().height.as_span();
        int const array_size = static_cast<int>(data_span.size());
        EMLD_ASSERT(array_size == static_cast<int>(m_image.size()),
                    "image and data sizes must match");
        for (int i = 0; i < array_size; ++i) {
            float d = data_span[i];
            d = std::clamp((d + 1.0f) / 2.0f, 0.0f, 1.0f);
            d = std::pow(d, 2.2f);
            m_image[i] = mix(dark_blue, light_blue, d);
        }
    }

    std::string name() const override {
        return "shallow_weaver_sim";
    }

    V2i preferred_window_size() const override {
        return V2i{m_simulation.parameters().resolution,
                   m_simulation.parameters().resolution};
    }

    void init_draw(int w, int h) override {
        SimSlab::init_draw(w, h);

        m_slab_draw_helper = std::make_unique<SlabDrawHelper>(
            m_image.data(),
            m_simulation.parameters().resolution,
            m_simulation.parameters().resolution);
    }

    void draw() override {
        if (m_slab_draw_helper) { m_slab_draw_helper->draw(); }
    }

    void step() override {
        m_simulation.step();
        copy_state_to_image();
        if (m_slab_draw_helper) { m_slab_draw_helper->update(m_image.data()); }
    }

    void mouse(int i_button,
               int i_action,
               int i_mods,
               double x,
               double y,
               double lastX,
               double lastY) override {
        if (i_button != GLFW_MOUSE_BUTTON_LEFT) { return; }

        if (i_action == GLFW_PRESS) {
            V2f const ndc_mouse_pos{
                std::clamp((static_cast<float>(x) + 0.5f) /
                               static_cast<float>(m_width),
                           0.0f,
                           1.0f),
                std::clamp(1.0f - (static_cast<float>(y) + 0.5f) /
                                      static_cast<float>(m_height),
                           0.0f,
                           1.0f)};

            m_simulation.set_input(ndc_mouse_pos, 1.5f);
            m_mouse_down = true;
        } else if (i_action == GLFW_RELEASE) {
            m_simulation.set_input();
            m_mouse_down = false;
        }
    }

    void mouse_drag(double x, double y, double lastX, double lastY) override {
        if (!m_mouse_down) { return; }

        V2f const ndc_mouse_pos{
            std::clamp(
                (static_cast<float>(x) + 0.5f) / static_cast<float>(m_width),
                0.0f,
                1.0f),
            std::clamp(1.0f - (static_cast<float>(y) + 0.5f) /
                                  static_cast<float>(m_height),
                       0.0f,
                       1.0f)};

        m_simulation.set_input(ndc_mouse_pos, 1.5f);
        m_mouse_down = true;
    }

private:
    Simulation m_simulation;
    std::vector<V3f> m_image;
    std::unique_ptr<SlabDrawHelper> m_slab_draw_helper;
    bool m_mouse_down = false;
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
    SimPtr sptr =
        std::make_shared<Shallow_weaver_sim>(parse_parameters(argc, argv));
    SimpleViewSim(sptr, true);
    return 0;
}
