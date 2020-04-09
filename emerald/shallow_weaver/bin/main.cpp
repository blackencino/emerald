#include <emerald/geep_glfw/program.h>
#include <emerald/shallow_weaver/bin/parse_parameters.h>
#include <emerald/shallow_weaver/simulation.h>
#include <emerald/simple_sim_viewer/viewer.h>
#include <emerald/util/assert.h>
#include <emerald/util/format.h>
#include <emerald/util/functions.h>

#include <fmt/format.h>

using namespace emerald::simple_sim_viewer;
using namespace emerald::util;
using namespace emerald::shallow_weaver;
using namespace emerald::geep_glfw;

//------------------------------------------------------------------------------
class Shallow_weaver_slab_draw {
public:
    Shallow_weaver_slab_draw(uint8_t const* const water_pixels,
                             float const water_min,
                             float const water_max,
                             uint8_t const* const terrain_pixels,
                             float const terrain_min,
                             float const terrain_max,
                             int const width,
                             int const height)
      : m_water_min(water_min)
      , m_water_max(water_max)
      , m_terrain_min(terrain_min)
      , m_terrain_max(terrain_max)
      , m_width(width)
      , m_height(height) {
        util_gl::CheckErrors(
            "shallow weaver slab draw helper init before anything");

        // Create and bind VAO
        glGenVertexArrays(1, &m_vertexArrayObject);
        util_gl::CheckErrors("glGenVertexArrays");
        EMLD_ASSERT(m_vertexArrayObject > 0, "Failed to create VAO");

        glBindVertexArray(m_vertexArrayObject);
        util_gl::CheckErrors("glBindVertexArray");

        // Create vertex buffer.
        glGenBuffers(1, &m_quadVertexBufferObject);
        util_gl::CheckErrors("glGenBuffers");
        EMLD_ASSERT(m_quadVertexBufferObject > 0, "Failed to create VBO");

        static const std::array quad_vertex_data = {V2f{-1.0f, -1.0f},
                                                    V2f{1.0f, -1.0f},
                                                    V2f{-1.0f, 1.0f},
                                                    V2f{1.0f, 1.0f}};

        // POS buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_quadVertexBufferObject);
        util_gl::CheckErrors("glBindBuffer POS");
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(V2f) * 4,
                     reinterpret_cast<GLvoid const*>(quad_vertex_data.data()),
                     GL_STATIC_DRAW);
        util_gl::CheckErrors("glBufferData POS");
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
        util_gl::CheckErrors("glVertexAttribPointer POS");
        glEnableVertexAttribArray(0);
        util_gl::CheckErrors("glEnableVertexAttribArray POS");

        // Unbind VAO.
        glBindVertexArray(0);
        util_gl::CheckErrors("Unbind VAO");

        // Make textures
        glGenTextures(2, m_textureObjects.data());
        util_gl::CheckErrors("glGenTextures");
        EMLD_ASSERT(m_textureObjects[0] > 0, "Failed to create Texture 0");
        EMLD_ASSERT(m_textureObjects[1] > 0, "Failed to create Texture 1");

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_textureObjects[0]);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glTexImage2D(GL_TEXTURE_2D,  // target
                     0,              // mip-map level
                     GL_RED,         // internal format
                     m_width,
                     m_height,
                     0,                 // border
                     GL_RED,            // format of incoming data
                     GL_UNSIGNED_BYTE,  // type of incoming data
                     reinterpret_cast<void const*>(water_pixels));
        util_gl::CheckErrors("glTexImage2D");
        glBindTexture(GL_TEXTURE_2D, 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, m_textureObjects[1]);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glTexImage2D(GL_TEXTURE_2D,  // target
                     0,              // mip-map level
                     GL_RED,         // internal format
                     m_width,
                     m_height,
                     0,                 // border
                     GL_RED,            // format of incoming data
                     GL_UNSIGNED_BYTE,  // type of incoming data
                     reinterpret_cast<void const*>(terrain_pixels));
        util_gl::CheckErrors("glTexImage2D");
        glBindTexture(GL_TEXTURE_2D, 0);

        static auto const* const vertex_source = R"(
        #version 150
        in vec2 position;
        out vec2 texcoord;
        void main() {
            gl_Position = vec4(position, 0.0, 1.0);
            texcoord = position * 0.5 + vec2(0.5, 0.5);
        }
    )";

        static auto const* const fragment_source = R"(
        #version 150
        in vec2 texcoord;
        out vec4 fragment_color;

        uniform sampler2D water_texture;
        uniform sampler2D terrain_texture;

        uniform float water_min;
        uniform float water_max;

        uniform float terrain_min;
        uniform float terrain_max;

        const vec3 dark_blue = vec3(0.01, 0.01, 0.2);
        const vec3 light_blue = vec3(0.9, 0.9, 1.0);

        float linstep(float edge0, float edge1, float t) {
            return clamp((t - edge0) / (edge1 - edge0), 0.0, 1.0);
        }

        vec3 cpow(vec3 c, float exponent) {
            return vec3(pow(c.x, exponent),
                        pow(c.y, exponent),
                        pow(c.z, exponent));
        }

        void main() {

            float water_height = texture(water_texture, texcoord).r;
            water_height = mix(water_min, water_max, water_height);
            float water_interp = linstep(-1.0, 1.0, water_height);
            vec3 water_color = mix(dark_blue, light_blue, water_interp);

            float terrain_height_01 = texture(terrain_texture, texcoord).r;
            float terrain_height = mix(terrain_min, terrain_max, terrain_height_01);
            vec3 terrain_color = mix(vec3(1, 0, 0), vec3(0, 1, 0), terrain_height_01);

            float terrain_mix = terrain_height - water_height;
            terrain_mix = smoothstep(0.0, 0.01, terrain_mix);

            vec3 final_color = mix(water_color, terrain_color, terrain_mix);

            fragment_color = vec4(cpow(final_color, 2.2), 1);
        }
    )";

        // Make program
        Program::Bindings const vtx_bindings = {{0, "position"}};
        Program::Bindings const frg_bindings = {{0, "fragment_color"}};
        m_program = std::make_unique<Program>("Draw quad helper",
                                              vertex_source,
                                              "",
                                              fragment_source,
                                              vtx_bindings,
                                              frg_bindings,
                                              m_vertexArrayObject);

        m_program->use();
        (*m_program)(Uniform{"water_texture", 0, Uniform::kRequireError});
        (*m_program)(Uniform{"terrain_texture", 1, Uniform::kRequireError});
        (*m_program)(Uniform{"water_min", m_water_min, Uniform::kRequireError});
        (*m_program)(Uniform{"water_max", m_water_max, Uniform::kRequireError});
        (*m_program)(
            Uniform{"terrain_min", m_terrain_min, Uniform::kRequireError});
        (*m_program)(
            Uniform{"terrain_max", m_terrain_max, Uniform::kRequireError});
        m_program->setUniforms();
        m_program->unuse();
    }

    ~Shallow_weaver_slab_draw() {
        if (m_vertexArrayObject > 0) {
            glDeleteVertexArrays(1, &m_vertexArrayObject);
            m_vertexArrayObject = 0;
        }

        if (m_quadVertexBufferObject > 0) {
            glDeleteBuffers(1, &m_quadVertexBufferObject);
            m_quadVertexBufferObject = 0;
        }

        if (m_textureObjects[0] > 0 || m_textureObjects[1] > 0) {
            glDeleteTextures(2, m_textureObjects.data());
            m_textureObjects = {0, 0};
        }
    }

    void update_water(uint8_t const* const pixels,
                      float const hmin,
                      float const hmax) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_textureObjects[0]);

        glTexSubImage2D(GL_TEXTURE_2D,  // target
                        0,              // mip-map level
                        0,              // x-offset
                        0,              // y-offset
                        m_width,
                        m_height,
                        GL_RED,            // format of incoming data
                        GL_UNSIGNED_BYTE,  // type of incoming data
                        reinterpret_cast<void const*>(pixels));
        util_gl::CheckErrors("glTexSubImage2D");
        glBindTexture(GL_TEXTURE_2D, 0);

        m_water_min = hmin;
        m_water_max = hmax;

        (*m_program)(Uniform{"water_min", m_water_min, Uniform::kRequireError});
        (*m_program)(Uniform{"water_max", m_water_max, Uniform::kRequireError});
    }

    void update_terrain(uint8_t const* const pixels,
                        float const tmin,
                        float const tmax) {
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, m_textureObjects[1]);

        glTexSubImage2D(GL_TEXTURE_2D,  // target
                        0,              // mip-map level
                        0,              // x-offset
                        0,              // y-offset
                        m_width,
                        m_height,
                        GL_RED,            // format of incoming data
                        GL_UNSIGNED_BYTE,  // type of incoming data
                        reinterpret_cast<void const*>(pixels));
        util_gl::CheckErrors("glTexSubImage2D");
        glBindTexture(GL_TEXTURE_2D, 0);

        m_terrain_min = tmin;
        m_terrain_max = tmax;

        (*m_program)(
            Uniform{"terrain_min", m_terrain_min, Uniform::kRequireError});
        (*m_program)(
            Uniform{"terrain_max", m_terrain_max, Uniform::kRequireError});
    }

    void draw() const {
        // Bind the vertex array
        glBindVertexArray(m_vertexArrayObject);
        util_gl::CheckErrors("glBindVertexArray draw");

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_textureObjects[0]);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, m_textureObjects[1]);

        m_program->use();
        m_program->setUniforms();

        // Draw the elements
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        util_gl::CheckErrors("glDrawArrays");

        m_program->unuse();

        // Unbind the vertex array
        glBindVertexArray(0);
        util_gl::CheckErrors("glBindVertexArray 0 draw");

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

private:
    float m_water_min = -1.0f;
    float m_water_max = 1.0f;
    float m_terrain_min = -10.0f;
    float m_terrain_max = 10.0f;
    int m_width = 0;
    int m_height = 0;

    GLuint m_vertexArrayObject = 0;
    GLuint m_quadVertexBufferObject = 0;
    std::array<GLuint, 2> m_textureObjects = {0, 0};
    std::unique_ptr<Program> m_program;
};

//------------------------------------------------------------------------------
class Shallow_weaver_sim : public SimSlab {
public:
    explicit Shallow_weaver_sim(Simulation::Parameters const& params)
      : SimSlab()
      , m_simulation(params) {
        m_water_image.resize(params.resolution * params.resolution, 128);
        m_terrain_image.resize(params.resolution * params.resolution, 128);
        copy_state_to_images();
    }

    void copy_state_to_images() {
        auto const water_data_span = m_simulation.state().height.as_span();
        int const water_array_size = static_cast<int>(water_data_span.size());
        EMLD_ASSERT(water_array_size == static_cast<int>(m_water_image.size()),
                    "water image and data sizes must match");
        m_water_min = -1.0f;
        m_water_max = 1.0f;
        for (int i = 0; i < water_array_size; ++i) {
            auto const d =
                linstep(m_water_min, m_water_max, water_data_span[i]);
            m_water_image[i] = static_cast<uint8_t>(255.0f * d);
        }

        auto const terrain_data_span =
            m_simulation.state().terrain_height.as_span();
        int const terrain_array_size =
            static_cast<int>(terrain_data_span.size());
        EMLD_ASSERT(
            terrain_array_size == static_cast<int>(m_terrain_image.size()),
            "terrain image and data sizes must match");
        float tmin = std::numeric_limits<float>::max();
        float tmax = -std::numeric_limits<float>::max();
        for (int i = 0; i < terrain_array_size; ++i) {
            auto const d = terrain_data_span[i];
            tmin = std::min(d, tmin);
            tmax = std::max(d, tmax);
        }
        float const tcen = (tmax + tmin) / 0.5f;
        float const trange = std::max(tmax - tmin, 1.0f);
        m_terrain_min = tcen - 0.5f * trange;
        m_terrain_max = tcen + 0.5f * trange;
        for (int i = 0; i < terrain_array_size; ++i) {
            auto const d =
                linstep(m_terrain_min, m_terrain_max, terrain_data_span[i]);
            m_terrain_image[i] = static_cast<uint8_t>(255.0f * d);
        }
    }

    std::string name() const override {
        return "shallow_weaver_sim";
    }

    V2i preferred_window_size() const override {
        return V2i{512, 512};
    }

    void init_draw(int w, int h) override {
        SimSlab::init_draw(w, h);

        m_slab_draw = std::make_unique<Shallow_weaver_slab_draw>(
            m_water_image.data(),
            m_water_min,
            m_water_max,
            m_terrain_image.data(),
            m_terrain_min,
            m_terrain_max,
            m_simulation.parameters().resolution,
            m_simulation.parameters().resolution);

        m_updated = true;
    }

    bool needs_redraw() override {
        return m_updated;
    }

    void draw() override {
        if (m_slab_draw) {
            m_slab_draw->draw();
            m_updated = false;
        }
    }

    void step() override {
        m_simulation.step();
        copy_state_to_images();
        if (m_slab_draw) {
            m_slab_draw->update_water(
                m_water_image.data(), m_water_min, m_water_max);
            m_slab_draw->update_terrain(
                m_terrain_image.data(), m_terrain_min, m_terrain_max);
            m_updated = true;
        }
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
    std::vector<uint8_t> m_water_image;
    float m_water_min = -1.0f;
    float m_water_max = 1.0f;
    std::vector<uint8_t> m_terrain_image;
    float m_terrain_min = -10.0f;
    float m_terrain_max = 10.0f;

    bool m_updated = false;

    std::unique_ptr<Shallow_weaver_slab_draw> m_slab_draw;
    bool m_mouse_down = false;
};

//-*****************************************************************************
int main(int argc, char* argv[]) {
    SimPtr sptr =
        std::make_shared<Shallow_weaver_sim>(parse_parameters(argc, argv));
    SimpleViewSim(sptr, true);
    return 0;
}
