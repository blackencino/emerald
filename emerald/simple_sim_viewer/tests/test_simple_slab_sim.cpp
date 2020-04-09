#include <emerald/simple_sim_viewer/slab_draw_helper.h>
#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/util/assert.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;

//-*****************************************************************************
class SimpleSlabSim : public SimSlab {
public:
    SimpleSlabSim()
      : SimSlab()
      , m_image_width(512)
      , m_image_height(512) {
        m_image.resize(m_image_width * m_image_height, V3f{0.0f});

        constexpr int check_size = 32;

        for (int j = 0; j < m_image_height; ++j) {
            auto const parity_j = (j / check_size) % 2;
            for (int i = 0; i < m_image_width; ++i) {
                auto const parity_i = (i / check_size) % 2;

                if (parity_i == parity_j) {
                    m_image[i + (j * m_image_width)] = V3f{1.0f};
                }
            }
        }
    }

    std::string name() const override {
        return "SimpleSlabSim";
    }

    V2i preferred_window_size() const override {
        return V2i{m_image_width, m_image_height};
    }

    void init_draw(int w, int h) override {
        SimSlab::init_draw(w, h);

        m_slabDrawHelper = std::make_unique<SlabDrawHelper>(
            m_image.data(), m_image_width, m_image_height);

        m_updated = true;
    }

    bool needs_redraw() override {
        return m_updated;
    }

    void draw() override {
        if (m_slabDrawHelper) {
            m_slabDrawHelper->draw();
            m_updated = false;
        }
    }

private:
    std::vector<V3f> m_image;
    int m_image_width = 0;
    int m_image_height = 0;
    bool m_updated = false;

    std::unique_ptr<SlabDrawHelper> m_slabDrawHelper;
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
    SimPtr sptr = std::make_shared<SimpleSlabSim>();
    SimpleViewSim(sptr, true);
    return 0;
}
